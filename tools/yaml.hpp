/*
 * info_relay - 串口与导航通信中转进程
 * Copyright (c) 2026 Navigator Vision Team
 *
 * 代码来源 / Source: Navigator Vision Team
 * 仓库 / Repository: https://github.com/skiwal/info_relay
 * 队徽 / Team logo: assets/logo.jpg
 * SPDX-License-Identifier: MIT
 */

#ifndef INFO_RELAY__TOOLS__YAML_HPP
#define INFO_RELAY__TOOLS__YAML_HPP

/*
 * YAML 读取的极简封装：只做「加载文件」与「按键取值」两件事。
 *
 * 数据流：load(path) 得到节点树 → require<T>()/value<T>() 逐键读取到配置结构体。
 *
 * 依赖与边界：只包 yaml-cpp，不含任何业务语义；不在这里打印与退出，
 *             失败一律抛 std::runtime_error，由 main() 统一捕获并落日志。
 *
 * 使用注意：取不到必需键就抛异常是刻意设计——配置缺项属于部署错误，
 *           静默取默认值会让机器带着错误参数上场比赛。
 */

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>

namespace info_relay
{

// 功能：加载 YAML 文件
// 输入：
//   path  文件路径
// 输出：根节点
// 备注：文件不存在或语法错误时抛 std::runtime_error，消息含原始原因
inline YAML::Node load(const std::string & path)
{
  try {
    return YAML::LoadFile(path);
  } catch (const std::exception & e) {
    throw std::runtime_error("加载配置失败 " + path + "：" + e.what());
  }
}

// 功能：判断键是否存在且非空
// 输入：
//   node  父节点
//   key   键名
// 输出：存在可转换的值时为 true
// 备注：YAML 的空值（`key:` 后无内容）视为不存在
inline bool has(const YAML::Node & node, const std::string & key)
{
  return static_cast<bool>(node) && static_cast<bool>(node[key]);
}

// 功能：读取必需键
// 输入：
//   node  父节点
//   key   键名
// 输出：转换后的值
// 备注：键缺失或类型不符时抛 std::runtime_error
template <typename T>
inline T require(const YAML::Node & node, const std::string & key)
{
  if (!has(node, key)) throw std::runtime_error("配置缺少必需项：" + key);
  try {
    return node[key].as<T>();
  } catch (const std::exception & e) {
    throw std::runtime_error("配置项 " + key + " 类型不符：" + e.what());
  }
}

// 功能：读取可选键
// 输入：
//   node     父节点
//   key      键名
//   fallback 键缺失或类型不符时使用的值
// 输出：转换后的值或 fallback
// 备注：类型不符也返回 fallback，保证单个别扭的配置项不会让进程起不来
template <typename T>
inline T value(const YAML::Node & node, const std::string & key, const T & fallback)
{
  if (!has(node, key)) return fallback;
  try {
    return node[key].as<T>();
  } catch (const std::exception &) {
    return fallback;
  }
}

}  // namespace info_relay

#endif  // INFO_RELAY__TOOLS__YAML_HPP
