/*
 * info_relay - 串口与导航通信中转进程
 * Copyright (c) 2026 Navigator Vision Team
 *
 * 代码来源 / Source: Navigator Vision Team
 * 仓库 / Repository: https://github.com/skiwal/info_relay
 * 队徽 / Team logo: assets/logo.jpg
 * SPDX-License-Identifier: MIT
 */

#include "IO/serial/serial_io.hpp"

#include <exception>

#include "tools/logger.hpp"

namespace info_relay::io
{

SerialIo::~SerialIo() { close(); }

bool SerialIo::init(const IoOptions & options)
{
  if (options.type != IoType::Serial || options.device.empty()) return false;
  options_ = options;
  return true;
}

bool SerialIo::open()
{
  try {
    serial_.setPort(options_.device);
    serial_.setBaudrate(options_.baud_rate);
    serial_.setBytesize(serial::eightbits);
    serial_.setParity(serial::parity_none);
    serial_.setStopbits(serial::stopbits_one);
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(static_cast<uint32_t>(options_.timeout_ms));
    serial_.setTimeout(timeout);
    serial_.open();
  } catch (const std::exception & e) {
    logger()->error("[SerialIo] 打开 {} 失败：{}", options_.device, e.what());
    return false;
  }
  opened_ = true;
  logger()->info("[SerialIo] 已打开 {} @ {}", options_.device, options_.baud_rate);
  return true;
}

size_t SerialIo::read(uint8_t * buffer, size_t size, int timeout_ms)
{
  if (!opened_) return 0;
  std::lock_guard<std::mutex> lock(read_mutex_);
  try {
    if (timeout_ms != options_.timeout_ms) {
      serial::Timeout timeout = serial::Timeout::simpleTimeout(static_cast<uint32_t>(timeout_ms));
      serial_.setTimeout(timeout);
      options_.timeout_ms = timeout_ms;
    }
    return serial_.read(buffer, size);
  } catch (const std::exception &) {
    return 0;
  }
}

size_t SerialIo::write(const uint8_t * data, size_t size)
{
  if (!opened_) return 0;
  std::lock_guard<std::mutex> lock(write_mutex_);
  try {
    return serial_.write(data, size);
  } catch (const std::exception & e) {
    logger()->warn("[SerialIo] 写串口失败：{}", e.what());
    return 0;
  }
}

void SerialIo::close()
{
  std::lock_guard<std::mutex> rlock(read_mutex_);
  std::lock_guard<std::mutex> wlock(write_mutex_);
  if (!opened_) return;
  try {
    if (serial_.isOpen()) serial_.close();
  } catch (...) {
  }
  opened_ = false;
}

std::string SerialIo::describe() const
{
  return "serial:" + options_.device + "@" + std::to_string(options_.baud_rate);
}

}  // namespace info_relay::io
