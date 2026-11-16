/*
 * info_relay - 串口与导航通信中转进程
 * Copyright (c) 2026 Navigator Vision Team
 *
 * 代码来源 / Source: Navigator Vision Team
 * 仓库 / Repository: https://github.com/skiwal/info_relay
 * 队徽 / Team logo: assets/logo.jpg
 * SPDX-License-Identifier: MIT
 */

/*
 * 运行期路径解析的实现：仓库根 -> 各默认目录。
 *
 * 解析顺序（先命中先用）：
 *   1. 环境变量 INFO_RELAY_HOME；
 *   2. /proc/self/exe 的符号链接：若可执行文件位于名为 build 的目录下，取其父目录；
 *   3. 当前工作目录（并打印一次 warn，提示结果可能不在仓库内）。
 *
 * 注意：契约见 paths.hpp，本文件只写实现要点。
 */
#include "tools/paths.hpp"

#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <vector>

namespace info_relay::paths
{
namespace
{
// 功能：读取可执行文件所在目录
// 输入：无
// 输出：目录路径；读取失败返回空 path
// 备注：/proc/self/exe 是 Linux 专有接口，本项目只跑 Linux，无需跨平台分支
std::filesystem::path executable_dir()
{
  std::vector<char> buffer(4096, '\0');
  const ssize_t n = ::readlink("/proc/self/exe", buffer.data(), buffer.size() - 1);
  if (n <= 0) return {};
  return std::filesystem::path(buffer.data()).parent_path();
}
}  // namespace

std::filesystem::path home()
{
  // 结果缓存：全进程只解析一次，避免每帧调用时反复读环境变量与符号链接
  static const std::filesystem::path cached = [] {
    if (const char * env = std::getenv("INFO_RELAY_HOME"); env != nullptr && env[0] != '\0') {
      return std::filesystem::path(env);
    }

    const auto exe_dir = executable_dir();
    if (!exe_dir.empty()) {
      // 构建产物在 <repo>/build 下，因此 build 的父目录就是仓库根
      if (exe_dir.filename() == "build") return exe_dir.parent_path();
      return exe_dir;
    }

    // 这里不能用 logger()：logger 的惰性初始化会反查 paths::logs()，形成递归
    std::fprintf(stderr, "[paths] 无法定位可执行文件，路径退化为当前工作目录\n");
    return std::filesystem::current_path();
  }();
  return cached;
}

std::filesystem::path logs() { return home() / "logs"; }

std::filesystem::path default_config() { return home() / "configs" / "info_relay.yaml"; }

std::filesystem::path resolve(const std::string & p)
{
  if (p.empty()) return home();
  const std::filesystem::path path(p);
  if (path.is_absolute()) return path;
  return home() / path;
}

}  // namespace info_relay::paths
