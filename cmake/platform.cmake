# info_relay - 串口与导航通信中转进程
# Copyright (c) 2026 Navigator Vision Team
#
# 代码来源 / Source: Navigator Vision Team
# 仓库 / Repository: https://github.com/skiwal/info_relay
# 队徽 / Team logo: assets/logo.jpg
# SPDX-License-Identifier: MIT

# =====================================================================================
#  平台与编译选项（info_relay 自有，不依赖任何外部工程）
#
#  职责：把目标架构归一化成一个短名，并统一编译选项。
#  对外提供：
#    SP_PROCESSOR   x86_64 / aarch64     内核架构名，日志与诊断用
#    SP_ARCH        amd64 / arm64        仓库内二进制目录名，将来放架构相关的第三方产物
#
#  注意：架构差异只在本文件收敛；其他 CMakeLists 不得直接判断 CMAKE_SYSTEM_PROCESSOR。
# =====================================================================================

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64|amd64)$")
  set(SP_PROCESSOR "x86_64")
  set(SP_ARCH "amd64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
  set(SP_PROCESSOR "aarch64")
  set(SP_ARCH "arm64")
else()
  message(FATAL_ERROR "[platform] 不支持的目标架构：${CMAKE_SYSTEM_PROCESSOR}")
endif()

message(STATUS "[platform] 目标架构 ${SP_PROCESSOR}（仓库内二进制目录 ${SP_ARCH}）")

# 编译选项：只开常用告警，不开 -Werror。第三方库（libserial、iceoryx2 头）会产生噪声，
# 用 SYSTEM 方式引入它们的头文件来屏蔽，而不是整体关掉告警。
add_compile_options(-Wall -Wextra -Wno-unused-parameter)

# 统一输出目录：产物集中在 build/ 下，install 前缀默认也落在仓库内，
# 避免默认装到 /usr/local 污染系统。
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "安装前缀（默认在构建目录内）" FORCE)
endif()
