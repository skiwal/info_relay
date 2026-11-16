#ifndef INFO_RELAY__COMMON__SOURCE_MODE_HPP
#define INFO_RELAY__COMMON__SOURCE_MODE_HPP

/*
 * 数据源模式：真实串口 / 内置虚拟下位机 / 原始字节回放。
 *
 * 数据流：配置 source_mode 字符串 → parse_source_mode() → make_transport() 选择实现。
 *
 * 依赖与边界：纯头文件、纯标准库；只描述"字节从哪来"，不含任何协议语义。
 *
 * 使用注意：日志标签只用 log_tag() 取，不要在别处再写一套字符串比较，
 *           否则新增模式时会漏改。
 */

#include <string>

namespace info_relay
{

// 字节流的来源
enum class SourceMode
{
  Real,    // 真实串口设备
  Fake,    // 内置虚拟下位机，或外部进程创建的 pty
  Replay,  // 原始字节录制文件回放
};

// 功能：把配置字符串解析成模式
// 输入：
//   s     配置值，允许空串
//   fallback  无法识别时使用的模式
// 输出：解析结果
// 备注：大小写敏感；空串与未知值都返回 fallback，不报错
inline SourceMode parse_source_mode(const std::string & s, SourceMode fallback)
{
  if (s == "real") return SourceMode::Real;
  if (s == "fake") return SourceMode::Fake;
  if (s == "replay") return SourceMode::Replay;
  return fallback;
}

// 功能：取模式的稳定字符串表示
// 输入：
//   mode  模式
// 输出：小写英文名，用于日志与配置回显
// 备注：与 parse_source_mode 严格互逆
inline const char * to_string(SourceMode mode)
{
  switch (mode) {
    case SourceMode::Real:
      return "real";
    case SourceMode::Fake:
      return "fake";
    case SourceMode::Replay:
      return "replay";
  }
  return "unknown";
}

// 功能：取日志标签
// 输入：
//   mode  模式
// 输出：大写标签，用于 prefixed 日志，便于过滤
// 备注：只影响日志前缀，不参与任何逻辑判断
inline const char * log_tag(SourceMode mode)
{
  switch (mode) {
    case SourceMode::Real:
      return "REAL";
    case SourceMode::Fake:
      return "FAKE";
    case SourceMode::Replay:
      return "REPLAY";
  }
  return "UNKNOWN";
}

}  // namespace info_relay

#endif  // INFO_RELAY__COMMON__SOURCE_MODE_HPP
