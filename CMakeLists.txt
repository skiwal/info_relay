# info_relay - 串口与导航通信中转进程
# Copyright (c) 2026 Navigator Vision Team
#
# 代码来源 / Source: Navigator Vision Team
# 仓库 / Repository: https://github.com/skiwal/info_relay
# 队徽 / Team logo: assets/logo.jpg
# SPDX-License-Identifier: MIT

# =====================================================================================
#  info_relay —— 通信中转进程（独立工程）
#
#  分层：src/ 转发主流程；tools/ 通用工具；IO/ 通信抽象与 serial/can 实现。
#  依赖：yaml-cpp、spdlog/fmt、Eigen3、iceoryx2（系统库）、3rdparty/serial。
#        刻意不依赖 ROS 2、OpenCV 与 FFmpeg —— 它们属于导航/视觉侧工程。
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

find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(iceoryx2-cxx REQUIRED)

message(STATUS "[info_relay] iceoryx2 配置目录 ${iceoryx2-cxx_DIR}")

add_subdirectory(3rdparty/serial)

# ---------------------------------------------------------------------------
# relay_tools：配置 / 日志 / 退出信号 / 路径解析
# ---------------------------------------------------------------------------
add_library(relay_tools STATIC
    tools/logger.cpp
    tools/exiter.cpp
    tools/paths.cpp
    tools/config.cpp
)
target_include_directories(relay_tools PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(relay_tools PUBLIC yaml-cpp Eigen3::Eigen Threads::Threads)

if(TARGET spdlog::spdlog)
  target_link_libraries(relay_tools PUBLIC spdlog::spdlog)
endif()
if(TARGET fmt::fmt)
  target_link_libraries(relay_tools PUBLIC fmt::fmt)
endif()

# ---------------------------------------------------------------------------
# relay_io：IO 抽象、serial/can 实现与各自协议
# ---------------------------------------------------------------------------
add_library(relay_io OBJECT
    IO/board.cpp
    IO/io_factory.cpp
    IO/serial/serial_io.cpp
    IO/serial/protocol/crc.cpp
    IO/serial/protocol/frame.cpp
    IO/serial/protocol/uplink_host.cpp
    IO/serial/protocol/uplink_referee.cpp
    IO/serial/protocol/downlink.cpp
    IO/serial/protocol/dispatch.cpp
    IO/can/can_io.cpp
    IO/can/protocol/crc.cpp
)
target_link_libraries(relay_io PUBLIC relay_tools serial)

# ---------------------------------------------------------------------------
# info_relay 进程：入口 + src 转发主流程
# ---------------------------------------------------------------------------
add_executable(info_relay
    main.cpp
    src/relay.cpp
)
target_link_libraries(info_relay PRIVATE
    relay_io
    relay_tools
    serial
    iceoryx2-cxx::static-lib-cxx
)

# ---------------------------------------------------------------------------
# 最小发布订阅自检：只验证 iceoryx2 依赖，不参与业务逻辑
# ---------------------------------------------------------------------------
add_executable(ipc_selftest tools/ipc_selftest.cpp)
target_link_libraries(ipc_selftest PRIVATE iceoryx2-cxx::static-lib-cxx)

install(TARGETS info_relay ipc_selftest RUNTIME DESTINATION bin)
install(FILES configs/info_relay.yaml DESTINATION configs)
