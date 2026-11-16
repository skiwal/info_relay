/*
 * info_relay - 串口与导航通信中转进程
 * Copyright (c) 2026 Navigator Vision Team
 *
 * 代码来源 / Source: Navigator Vision Team
 * 仓库 / Repository: https://github.com/skiwal/info_relay
 * 队徽 / Team logo: assets/logo.jpg
 * SPDX-License-Identifier: MIT
 */

#ifndef INFO_RELAY__IO__INC__IO_HPP
#define INFO_RELAY__IO__INC__IO_HPP

/*
 * 通信方式抽象：relay 只依赖本接口，不关心底层是串口还是 CAN。
 *
 * 生命周期：init() → open() → read()/write() 循环 → close()。
 * 线程模型：read()/write() 允许被不同线程调用，实现类必须保证线程安全。
 * 数据语义：read()/write() 均操作未经解析的字节流；分帧、CRC 与业务结构由
 *           serial/protocol 或 can/protocol 处理。
 */

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace info_relay::io
{

// 通信链路类型，由配置决定
enum class IoType : uint8_t
{
  Serial = 0,
  Can = 1,
};

// 与具体通信方式无关的初始化参数
struct IoOptions
{
  IoType type = IoType::Serial;  // 链路类型
  std::string device;            // 串口设备路径或 CAN 网络接口名
  uint32_t baud_rate = 921600;   // 串口波特率；CAN 忽略
  uint32_t can_id = 0x123;       // CAN 发送 ID；串口忽略
  int timeout_ms = 50;           // 单次 read 的等待上限
};

// 通信方式接口模板
class IO
{
public:
  using Ptr = std::unique_ptr<IO>;

  virtual ~IO() = default;

  // 功能：保存并校验初始化参数，不打开硬件
  // 输入：options 通信参数
  // 输出：true 表示参数合法
  virtual bool init(const IoOptions & options) = 0;

  // 功能：打开设备，失败时返回 false 并记录日志
  virtual bool open() = 0;

  // 功能：最多读取 size 字节，最多等待 timeout_ms
  // 输出：实际读取字节数；0 表示超时或无数据
  virtual size_t read(uint8_t * buffer, size_t size, int timeout_ms) = 0;

  // 功能：写出 size 字节
  // 输出：实际写入字节数
  virtual size_t write(const uint8_t * data, size_t size) = 0;

  // 功能：关闭设备并释放资源
  virtual void close() = 0;

  // 功能：返回可读的设备描述，用于日志
  virtual std::string describe() const = 0;
};

// 按配置创建通信实现；未知类型返回空指针
IO::Ptr make_io(const IoOptions & options);

}  // namespace info_relay::io

#endif  // INFO_RELAY__IO__INC__IO_HPP
