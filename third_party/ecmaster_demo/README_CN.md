# EcMasterDemoDc（Motrotech 示例）——它是怎么让电机动起来的

> 目标：让你**不需要猜**，就能从代码里看懂：主站每周期做了什么、PDO 是怎么映射的、CiA402 状态机怎么把驱动推进到可运行、以及目标位置/速度怎么写进从站。

---

## 0. 先说结论：电机“动起来”需要同时满足 3 个条件

1. **主站必须进入 OP**  
   只有在 OP（或 SAFEOP）阶段，`EcMasterJobTask()` 才会周期性调用 `myAppWorkpd()`，也才会周期性收发 PDO。

2. **必须把关键对象映射进 PDO**（否则指针为 `EC_NULL`，写不进去）  
   典型需要：  
   - 输出（主站→从站）：`0x6040 ControlWord`、`0x6060 ModeOfOperation`、`0x607A TargetPosition`、`0x60FF TargetVelocity`  
   - 输入（从站→主站）：`0x6041 StatusWord`、`0x6064 ActualPosition`（还有可选 `0x606C` 等）

3. **CiA402 状态机必须推进到 Operation Enabled**  
   仅仅写 TargetPosition/Velocity 不够；驱动必须在 `0x6041` 表示 “Operation enabled” 时才会执行运动命令。

---

## 1. 文件/函数总览（你从这里开始读就不会迷路）

### 1.1 `EcDemoMain.cpp`（程序入口）
只做“壳层”：命令行解析、日志、定时任务，然后调用：

- `EcDemoApp(&AppContext)`

### 1.2 `EcDemoApp.cpp`（主站生命周期 + demo 主循环）
最关键的阶段：

1) 初始化主站：`ecatInitMaster()`  
2) 配网/加载 ENI：`ecatConfigureNetwork(...)`  
3) DC/DCM 配置（可选）：`ecatDcConfigure()` / `ecatDcmConfigure()`  
4) 主站状态切换：`INIT -> PREOP -> SAFEOP -> OP`  
5) 启动周期线程：`EcMasterJobTask()`  
6) 应用回调：
   - `myAppPrepare()`：填写 `My_Slave[]` 站地址/轴数，并调用 `MT_Prepare()`
   - `myAppSetup()`：调用 `MT_Setup()` 完成 PDO 指针映射
   - `myAppWorkpd()`：每周期调用 `MT_Workpd()`

### 1.3 `motrotech.cpp`（伺服控制示例：PDO 指针 + 状态机 + 目标生成）
3 个核心函数：

- `MT_Setup()`：**把对象索引映射到 ProcessImage 指针**
- `Process_Commands()`：**CiA402 状态机（读 0x6041，写 0x6040）**
- `MT_Workpd()`：**每周期写 0x6060/0x607A/0x60FF，让电机按目标运动**

---

## 2. 最重要的一条链路：每个周期到底发生了什么？

### 2.1 周期线程 `EcMasterJobTask()` 的固定顺序（理解 PDO 的关键）
每周期（1ms/2ms/...）大致执行：

1. `ecatExecJob(eUsrJob_ProcessAllRxFrames)`  
   - 处理从站发来的 cyclic 帧  
   - 更新“输入过程数据（PdIn）”内存

2. `myAppWorkpd()`  
   - 读取 PdIn（例如 StatusWord/ActualPosition）  
   - 计算并写 PdOut（ControlWord/TargetPosition/TargetVelocity）

3. `ecatExecJob(eUsrJob_SendAllCycFrames)`  
   - 把 PdOut 打包成 cyclic 帧发给从站  
   - 从站收到后执行控制

所以“你写的目标”真正生效，是在 **SendAllCycFrames** 之后。

---

## 3. PDO 映射：为什么代码里全是指针？

### 3.1 ProcessImage 是什么？
EC‑Master 内部有两块连续内存：
- `PdOut`：主站写 → 从站读
- `PdIn`：从站写 → 主站读

### 3.2 `MT_Setup()` 做了什么？
它会查询 ENI 配置后得到每个 PDO 变量的 **位偏移 `nBitOffs`**，然后：

- `指针 = base + nBitOffs/8`

例如：
- `My_Motor[i].pwControlWord` 指向 PdOut 里 0x6040 所在位置
- `My_Motor[i].pwStatusWord` 指向 PdIn 里 0x6041 所在位置

如果 ENI 没把 0x6040 映射到 PDO，那么 `pwControlWord` 就会一直是 `EC_NULL`，驱动当然不会动。

---

## 4. CiA402 状态机：为什么要先写 ControlWord？

### 4.1 最简理解
- `0x6041 StatusWord`：驱动告诉你它现在在哪个状态（未就绪/可上电/已上电/已使能/故障）
- `0x6040 ControlWord`：你给驱动发“下一步该做什么”（shutdown/switch on/enable op/fault reset）

### 4.2 `Process_Commands()` 的工作方式（本 demo 的简化版）
每周期对每个轴：

1) 根据上层命令（`COMMAND_START`）决定目标态：`OP_ENABLED`  
2) 读取 `StatusWord` 判定当前态 `wActState`  
3) 如果未到目标态，就写对应控制字推进状态：
   - SwitchOnDisabled/NotReady → 写 `Shutdown (0x0006)`
   - ReadyToSwitchOn → 写 `SwitchOn (0x0007)`
   - SwitchedOn → 写 `EnableOperation (0x000F)`
   - Fault → 写 `FaultReset (0x0080)`（demo 里还夹杂计数节拍）

当 `StatusWord` 进入 `Operation enabled` 后，`MT_Workpd()` 才开始写目标位置/速度。

---

## 5. 目标是怎么生成的？为什么是“来回运动”？

`MT_Workpd()` 里实现了一个 demo 级的轨迹生成：
- 维护内部 `fCurVel`、`fCurPos`
- 用一个梯形速度曲线在正向/反向之间切换
- 最终写入：
  - `TargetPosition = fCurPos * INC_PERMM`
  - `TargetVelocity = fCurVel * INC_PERMM`

如果你要改成“点动/指定位置/外部给定”，就改这一段目标生成逻辑即可。

---

## 6. 最常见“为什么不动”的排查清单（非常实用）

1) **站地址是否匹配？**  
   `EcDemoApp.cpp` 写死 1001/1002，你实际从站固定站地址要一致。

2) **PDO 是否映射了 0x6040/0x6041/0x6060/0x607A/0x6064？**  
   没映射就不会有指针 → 写不进去 → 不会动。

3) **主站有没有进 OP？**  
   如果卡在 SAFEOP，多半是 DC/DCM 同步问题或 ENI 配置问题。

4) **驱动是否处于 Fault？**  
   demo 的 FaultReset 很简化，某些伺服还需要额外条件（上电/使能条件/外部 IO）。

---

## 7. 你给我 2 个信息，我就能帮你定位“为什么不动/该改哪”

1) 伺服从站实际固定站地址（例如 1001？）与轴数  
2) ENI 里该轴 PDO 映射（截图或复制出包含 0x6040/0x6041/0x607A 的片段）

