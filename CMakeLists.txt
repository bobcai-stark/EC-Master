
# cmake version
cmake_minimum_required(VERSION 3.28.3)

# project name
project(BasicService VERSION 1.0.0.0 LANGUAGES CXX)

# 查找要编译的.cpp文件
# search source files for .cpp 
file(GLOB_RECURSE OTA_ENGINE_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

# 构建时间
# build time
string(TIMESTAMP APP_BUILD_TIME "%Y-%m-%d %H:%M:%S")

# 根据version.h.in模板生产version.h头文件
# generate version.h from version.h.in
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/version.h.in ${CMAKE_CURRENT_SOURCE_DIR}/version.h)

# EC-Master SDK 位置可通过 -DECM_SDK_ROOT 覆盖
set(ECM_SDK_ROOT "/home/cai/EC-Master/SDK" CACHE PATH "Path to EC-Master SDK root")
set(ECM_SDK_LIB_DIR "${ECM_SDK_ROOT}/LIB/Linux/aarch64")

add_subdirectory(third_party/ecmaster_demo)

add_executable(${PROJECT_NAME} ${OTA_ENGINE_SRC})
message("app PROJECT_NAME:" ${PROJECT_NAME})

include(FetchContent)
# add dependence library tiny_framework
FetchContent_Declare(tiny_framework
    GIT_REPOSITORY https://github.com/New-Stark-Industries/tiny_framework.git
    GIT_TAG master)
FetchContent_MakeAvailable(tiny_framework)

target_link_libraries(${PROJECT_NAME} PRIVATE
    tiny_framework
    ecmaster_demo
    EcMaster
    EcMasterRasServer
    pthread
    m
    dl
    rt
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # 构建时使用源码目录的 include
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}> # 安装后使用安装目录的 include
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    ${ECM_SDK_ROOT}/INC
    ${ECM_SDK_ROOT}/INC/Linux
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ecmaster_demo
)
