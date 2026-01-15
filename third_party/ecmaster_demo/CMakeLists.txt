cmake_minimum_required(VERSION 3.16)

# 指向外部安装的 EC-Master SDK 根目录，可在配置时通过
# -DECM_SDK_ROOT=/path/to/SDK 覆盖。默认值根据当前开发机设置。
set(ECM_SDK_ROOT "/home/cai/EC-Master/SDK" CACHE PATH "Path to EC-Master SDK root")
# 部分头文件（如 EcTimer.h）位于 Sources 目录，独立配置以便覆盖
set(ECM_SOURCE_ROOT "/home/cai/EC-Master/Sources" CACHE PATH "Path to EC-Master Sources root")
set(ECM_SDK_LIB_DIR "${ECM_SDK_ROOT}/LIB/Linux/aarch64")

set(ECM_SOURCES
    EcDemoApp.cpp
    motrotech.cpp
    Common/EcDemoParms.cpp
    Common/EcDemoTimingTask.cpp
    Common/EcLogging.cpp
    Common/EcNotification.cpp
    Common/EcSdoServices.cpp
    Common/EcSelectLinkLayer.cpp
    Common/EcSlaveInfo.cpp
    Common/Linux/EcDemoMain.cpp
    Common/Linux/EcDemoTimingTaskPlatform.cpp
)

add_library(ecmaster_demo STATIC ${ECM_SOURCES})

target_include_directories(ecmaster_demo PUBLIC
    ${ECM_SDK_ROOT}/INC
    ${ECM_SDK_ROOT}/INC/Linux
    ${ECM_SOURCE_ROOT}/Common
    ${ECM_SOURCE_ROOT}/OsAbstraction
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Common
    ${CMAKE_CURRENT_SOURCE_DIR}/Common/Linux
)

target_link_directories(ecmaster_demo PUBLIC ${ECM_SDK_LIB_DIR})

target_link_libraries(ecmaster_demo
    EcMaster
    EcMasterRasServer
    pthread
    m
    dl
    rt
)
