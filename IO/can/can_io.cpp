/*
 * info_relay - 串口与导航通信中转进程
 * Copyright (c) 2026 Navigator Vision Team
 *
 * 代码来源 / Source: Navigator Vision Team
 * 仓库 / Repository: https://github.com/skiwal/info_relay
 * 队徽 / Team logo: assets/logo.jpg
 * SPDX-License-Identifier: MIT
 */

#include "IO/can/can_io.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <exception>

#include "IO/can/protocol/downlink.hpp"
#include "tools/logger.hpp"

namespace info_relay::io
{

CanIo::~CanIo() { close(); }

bool CanIo::init(const IoOptions & options)
{
  if (options.type != IoType::Can || options.device.empty()) return false;
  options_ = options;
  return true;
}

bool CanIo::open()
{
  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    logger()->error("[CanIo] 创建 SocketCAN 套接字失败：{}", std::strerror(errno));
    return false;
  }

  try {
    struct ifreq req {};
    std::strncpy(req.ifr_name, options_.device.c_str(), IFNAMSIZ - 1);
    if (::ioctl(fd_, SIOCGIFINDEX, &req) < 0) {
      logger()->error("[CanIo] 找不到 CAN 接口 {}：{}", options_.device, std::strerror(errno));
      ::close(fd_);
      fd_ = -1;
      return false;
    }

    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = req.ifr_ifindex;
    if (::bind(fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      logger()->error("[CanIo] 绑定 CAN 接口 {} 失败：{}", options_.device, std::strerror(errno));
      ::close(fd_);
      fd_ = -1;
      return false;
    }
  } catch (const std::exception & e) {
    logger()->error("[CanIo] 打开 {} 异常：{}", options_.device, e.what());
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  opened_ = true;
  logger()->info("[CanIo] 已打开 {}，发送 ID 0x{:03X}", options_.device, options_.can_id);
  return true;
}

bool CanIo::consume_frame(const can::protocol::CanFrame & frame)
{
  if (frame.dlc != 8) return false;
  auto chunk = can::protocol::decode_chunk(frame.data.data());
  if (!can::protocol::chunk_valid(chunk)) return false;
  if (chunk.seq == last_rx_seq_) return false;

  auto & group = pending_[chunk.seq];
  group[chunk.index] = chunk;
  if (group.size() != chunk.count) return false;

  std::vector<uint8_t> assembled;
  for (uint8_t i = 0; i < chunk.count; ++i) {
    const auto it = group.find(i);
    if (it == group.end()) return false;
    const auto & part = it->second;
    assembled.insert(assembled.end(), part.data, part.data + part.len);
  }

  last_rx_seq_ = chunk.seq;
  pending_.erase(chunk.seq);
  rx_bytes_.insert(rx_bytes_.end(), assembled.begin(), assembled.end());
  return true;
}

size_t CanIo::read(uint8_t * buffer, size_t size, int timeout_ms)
{
  if (!opened_) return 0;
  {
    std::lock_guard<std::mutex> lock(rx_mutex_);
    if (!rx_bytes_.empty()) {
      const size_t n = std::min(size, rx_bytes_.size());
      std::copy(rx_bytes_.begin(), rx_bytes_.begin() + static_cast<std::ptrdiff_t>(n), buffer);
      rx_bytes_.erase(rx_bytes_.begin(), rx_bytes_.begin() + static_cast<std::ptrdiff_t>(n));
      return n;
    }
  }

  struct pollfd pfd {};
  pfd.fd = fd_;
  pfd.events = POLLIN;
  const int ready = ::poll(&pfd, 1, timeout_ms);
  if (ready <= 0 || !(pfd.revents & POLLIN)) return 0;

  struct can_frame frame {};
  const ssize_t n = ::read(fd_, &frame, sizeof(frame));
  if (n != static_cast<ssize_t>(sizeof(frame))) return 0;

  std::lock_guard<std::mutex> lock(rx_mutex_);
  can::protocol::CanFrame converted;
  converted.id = frame.can_id;
  converted.dlc = frame.can_dlc;
  std::copy(std::begin(frame.data), std::end(frame.data), converted.data.begin());
  consume_frame(converted);

  const size_t out = std::min(size, rx_bytes_.size());
  std::copy(rx_bytes_.begin(), rx_bytes_.begin() + static_cast<std::ptrdiff_t>(out), buffer);
  rx_bytes_.erase(rx_bytes_.begin(), rx_bytes_.begin() + static_cast<std::ptrdiff_t>(out));
  return out;
}

size_t CanIo::write(const uint8_t * data, size_t size)
{
  if (!opened_ || size == 0) return 0;
  std::lock_guard<std::mutex> lock(tx_mutex_);

  can::protocol::CanDownlinkMessage message;
  message.seq = tx_seq_++;
  size = std::min(size, message.payload.size());
  std::copy(data, data + size, message.payload.begin());
  message.size = size;

  const uint8_t count = static_cast<uint8_t>((size + 2) / 3);
  for (uint8_t i = 0; i < count; ++i) {
    can::protocol::CanChunk chunk;
    chunk.seq = message.seq;
    chunk.index = i;
    chunk.count = count;
    chunk.len = static_cast<uint8_t>(std::min<size_t>(3, size - i * 3));
    std::copy(
      message.payload.begin() + i * 3,
      message.payload.begin() + i * 3 + chunk.len,
      chunk.data);

    uint8_t raw[8];
    can::protocol::encode_chunk(chunk, raw);

    struct can_frame frame {};
    frame.can_id = options_.can_id & 0x7FFU;
    frame.can_dlc = 8;
    std::copy(raw, raw + 8, frame.data);
    if (::write(fd_, &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame))) {
      logger()->warn("[CanIo] 发送分片 {}/{} 失败", i + 1, count);
      return i * 3;
    }
  }
  return size;
}

void CanIo::close()
{
  std::lock_guard<std::mutex> rlock(rx_mutex_);
  std::lock_guard<std::mutex> wlock(tx_mutex_);
  if (fd_ < 0) return;
  ::close(fd_);
  fd_ = -1;
  opened_ = false;
  pending_.clear();
  rx_bytes_.clear();
}

std::string CanIo::describe() const
{
  return "can:" + options_.device + "@0x" + std::to_string(options_.can_id);
}

}  // namespace info_relay::io
