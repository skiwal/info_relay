# info_relay - 串口与导航通信中转进程
# Copyright (c) 2026 Navigator Vision Team
#
# 代码来源 / Source: Navigator Vision Team
# 仓库 / Repository: https://github.com/skiwal/info_relay
# 队徽 / Team logo: assets/logo.jpg
# SPDX-License-Identifier: MIT

# =====================================================================================
#  info_relay —— 串口与导航通信中转进程（独立工程）
#
#  职责：独占串口、解析下位机协议，通过 iceoryx2 对外发布；整机唯一打开串口的程序。
#
#  依赖：yaml-cpp（配置）、spdlog/fmt（日志）、iceoryx2（跨进程，系统库）、
#        libserial（模块内 3rdparty/serial）。
#        刻意不依赖 ROS 2、OpenCV 与 FFmpeg —— 它们属于导航/视觉侧工程。
#
#  构建：cmake -S . -B build && cmake --build build
#  产物：build/info_relay、build/ipc_selftest；运行日志在 <repo>/logs，
#        串口录制在 <repo>/records，均不出仓库目录。
# =====================================================================================
cmake_minimum_required(VERSION 3.16)

project(info_relay VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "构建类型" FORCE)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(platform)

# ---------------------------------------------------------------------------
# 第三方依赖：全部走系统包，不在仓库内 vendor 二进制
# ---------------------------------------------------------------------------
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(iceoryx2-cxx REQUIRED)

message(STATUS "[info_relay] iceoryx2 配置目录 ${iceoryx2-cxx_DIR}")

# 串口字节库：随模块自带，只服务本工程
add_subdirectory(3rdparty/serial)

# ---------------------------------------------------------------------------
# relay_common：日志 / YAML / 信号退出 / 路径解析 / 配置加载
# ---------------------------------------------------------------------------
add_library(relay_common STATIC
    common/logger.cpp
    common/exiter.cpp
    common/paths.cpp
    config.cpp
)
target_include_directories(relay_common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(relay_common PUBLIC yaml-cpp Eigen3::Eigen)

# spdlog 在多数发行版以 header-only 形式提供，编译版才需要显式链接
if(TARGET spdlog::spdlog)
  target_link_libraries(relay_common PUBLIC spdlog::spdlog)
endif()
if(TARGET fmt::fmt)
  target_link_libraries(relay_common PUBLIC fmt::fmt)
endif()

# ---------------------------------------------------------------------------
# relay_link：字节传输 + 分帧 + 协议解析 + 串口字节记录
# ---------------------------------------------------------------------------
add_library(relay_link OBJECT
    # 收发线程与状态搬运
    link/serial_link.cpp
    # 分帧与号段分派
    link/protocol/link.cpp
    # 0x05xx 上行（host）与裁判转发（赛季协议）
    link/protocol/host_protocol.cpp
    link/protocol/referee.cpp
    link/protocol/seasons/2027/protocol.cpp
    # 下行报文与 CRC
    link/protocol/gimbal/protocol_crc.cpp
    # 数据源：真实串口 / 内置虚拟下位机 / 回放
    link/transport/transport.cpp
    link/transport/virtual_mcu.cpp
    link/transport/quaternion_source.cpp
    link/transport/serial_recorder.cpp
)
target_link_libraries(relay_link PUBLIC relay_common serial)

# ---------------------------------------------------------------------------
# relay 进程
# ---------------------------------------------------------------------------
add_executable(info_relay
    main.cpp
    relay.cpp
)
target_link_libraries(info_relay PRIVATE
    relay_link
    relay_common
    serial
    iceoryx2-cxx::static-lib-cxx
)

# ---------------------------------------------------------------------------
# 最小发布订阅自检：验证依赖与共享内存通路，不参与业务逻辑
# ---------------------------------------------------------------------------
add_executable(ipc_selftest selftest/ipc_selftest.cpp)
target_link_libraries(ipc_selftest PRIVATE iceoryx2-cxx::static-lib-cxx)

install(TARGETS info_relay ipc_selftest RUNTIME DESTINATION bin)
install(FILES configs/info_relay.yaml DESTINATION configs)
