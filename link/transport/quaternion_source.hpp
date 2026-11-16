/*
 * info_relay - 串口与导航通信中转进程
 * Copyright (c) 2026 Navigator Vision Team
 *
 * 代码来源 / Source: Navigator Vision Team
 * 仓库 / Repository: https://github.com/skiwal/info_relay
 * 队徽 / Team logo: assets/logo.jpg
 * SPDX-License-Identifier: MIT
 */

#ifndef IO__SIM__QUATERNION_SOURCE_HPP
#define IO__SIM__QUATERNION_SOURCE_HPP

// 四元数数据源抽象：业务算法只依赖此接口，不关心数据来自真实下位机、内置伪数据还是回放。
// Real/Fake/Replay 三种实现共用同一套“真实协议字节解析”，因此格式、频率、时间戳语义完全一致。

#include <Eigen/Geometry>

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "config.hpp"
#include "link/transport/serial_recorder.hpp"
#include "link/transport/transport.hpp"
#include "common/queue.hpp"

namespace info_relay::sim
{

struct QuaternionSample
{
  TimePoint timestamp{};                            // 统一单调时钟时间戳
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  bool valid = false;                               // 是否有效（校验通过）
  uint64_t seq = 0;                                 // 递增序号
  bool injected = false;                            // 是否来自注入的异常帧
};

class QuaternionSource
{
public:
  using Ptr = std::unique_ptr<QuaternionSource>;
  virtual ~QuaternionSource() = default;

  virtual void open() = 0;
  virtual bool read(QuaternionSample & out, int timeout_ms = 100) = 0;
  virtual void close() = 0;
  virtual std::string name() const = 0;
  virtual SourceMode mode() const = 0;
};

// 统一的字节流解析实现（0xA5 帧头 + CRC8 + CRC16 + 0x0503 云台姿态 → 四元数）
class SerialQuaternionSource : public QuaternionSource
{
public:
  ~SerialQuaternionSource() override;

  void open() override;
  bool read(QuaternionSample & out, int timeout_ms) override;
  void close() override;
  std::string name() const override { return name_; }
  SourceMode mode() const override { return mode_; }

  void attach_recorder(const std::shared_ptr<SerialRecorder> & recorder) { recorder_ = recorder; }

protected:
  SerialQuaternionSource(IByteTransport::Ptr transport, SourceMode mode, std::string name, bool external);

private:
  void parse_loop();
  void push_sample(const Eigen::Quaterniond & q);

  IByteTransport::Ptr transport_;
  SourceMode mode_;
  std::string name_;
  bool external_;

  std::thread thread_;
  std::atomic<bool> quit_{false};
  std::atomic<uint64_t> seq_{0};
  Queue<QuaternionSample> queue_{2000};
  std::shared_ptr<SerialRecorder> recorder_;
  bool closed_ = false;

  long long packet_count_ = 0;
  long long imu_count_ = 0;
};

// 真实串口四元数源
class RealSerialSource : public SerialQuaternionSource
{
public:
  explicit RealSerialSource(const RelayConfig & config);
};

// 内置伪数据四元数源（可注入异常）
class FakeSerialSource : public SerialQuaternionSource
{
public:
  explicit FakeSerialSource(const RelayConfig & config);
};

// 原始字节回放四元数源
class ReplaySerialSource : public SerialQuaternionSource
{
public:
  explicit ReplaySerialSource(const RelayConfig & config);
};

// 按配置返回 real/fake/replay 四元数源
QuaternionSource::Ptr make_quaternion_source(const RelayConfig & config);

inline RelayConfig with_serial_mode(const RelayConfig & config, SourceMode mode)
{
  RelayConfig copy = config;
  copy.serial.mode = mode;
  return copy;
}

}  // namespace info_relay::sim

#endif  // IO__SIM__QUATERNION_SOURCE_HPP
