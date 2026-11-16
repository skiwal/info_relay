/*
 * info_relay - 串口与导航通信中转进程
 * Copyright (c) 2026 Navigator Vision Team
 *
 * 代码来源 / Source: Navigator Vision Team
 * 仓库 / Repository: https://github.com/skiwal/info_relay
 * 队徽 / Team logo: assets/logo.jpg
 * SPDX-License-Identifier: MIT
 */

#ifndef INFO_RELAY__TOOLS__COMM_WARNING_HPP
#define INFO_RELAY__TOOLS__COMM_WARNING_HPP

/*
 * 通信异常集中告警：
 *   1. 错误发生点只做原子计数和少量上下文记录，不直接写日志、不抛异常、不中断流程；
 *   2. relay 主循环周期性 flush，输出一条或多条 warning，避免逐包刷屏；
 *   3. 所有计数累计保留，日志同时给出 total 与自上次汇报以来的 new。
 */

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <mutex>

#include "tools/logger.hpp"

namespace info_relay
{

enum class CommWarningKind : uint8_t
{
  Crc8Failure = 0,
  Crc16Failure,
  PayloadTooLong,
  LengthMismatch,
  UnknownCommand,
  CanBadDlc,
  CanChunkCrcFailure,
  CanDuplicateChunk,
  CanIncompleteChunk,
  IpcReceiveFailure,
  IpcLoanFailure,
  IpcSendFailure,
  SerialReadFailure,
  Count,
};

class CommWarnings
{
public:
  static constexpr auto kDefaultInterval = std::chrono::seconds(1);
  static constexpr size_t kContextSize = 48;

  void report(
    CommWarningKind kind,
    uint16_t cmd_id = 0,
    uint32_t expected = 0,
    uint32_t actual = 0,
    const char * context = nullptr)
  {
    if (kind >= CommWarningKind::Count) return;
    auto & slot = slots_[static_cast<size_t>(kind)];

    slot.count.fetch_add(1, std::memory_order_relaxed);
    total_.fetch_add(1, std::memory_order_relaxed);
    slot.last_ns.store(now_ns(), std::memory_order_relaxed);
    slot.cmd_id.store(cmd_id, std::memory_order_relaxed);
    slot.expected.store(expected, std::memory_order_relaxed);
    slot.actual.store(actual, std::memory_order_relaxed);

    {
      std::lock_guard<std::mutex> lock(slot.context_mutex);
      slot.context[0] = '\0';
      if (context != nullptr) {
        std::strncpy(slot.context, context, kContextSize - 1);
        slot.context[kContextSize - 1] = '\0';
      }
    }
  }

  uint64_t count(CommWarningKind kind) const
  {
    if (kind >= CommWarningKind::Count) return 0;
    return slots_[static_cast<size_t>(kind)].count.load(std::memory_order_relaxed);
  }

  uint64_t total() const { return total_.load(std::memory_order_relaxed); }

  void flush_if_due(std::chrono::nanoseconds interval = kDefaultInterval)
  {
    const uint64_t now = now_ns();
    const uint64_t last = last_flush_ns_.load(std::memory_order_relaxed);
    if (now - last < static_cast<uint64_t>(interval.count())) return;
    flush_at(now);
  }

  void flush() { flush_at(now_ns()); }

private:
  struct Slot
  {
    std::atomic<uint64_t> count{0};
    std::atomic<uint64_t> reported{0};
    std::atomic<uint64_t> last_ns{0};
    std::atomic<uint16_t> cmd_id{0};
    std::atomic<uint32_t> expected{0};
    std::atomic<uint32_t> actual{0};
    char context[kContextSize]{};
    std::mutex context_mutex;
  };

  static uint64_t now_ns()
  {
    return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch())
        .count());
  }

  static const char * name(CommWarningKind kind)
  {
    switch (kind) {
      case CommWarningKind::Crc8Failure: return "CRC8_FAILURE";
      case CommWarningKind::Crc16Failure: return "CRC16_FAILURE";
      case CommWarningKind::PayloadTooLong: return "PAYLOAD_TOO_LONG";
      case CommWarningKind::LengthMismatch: return "LENGTH_MISMATCH";
      case CommWarningKind::UnknownCommand: return "UNKNOWN_COMMAND";
      case CommWarningKind::CanBadDlc: return "CAN_BAD_DLC";
      case CommWarningKind::CanChunkCrcFailure: return "CAN_CHUNK_CRC_FAILURE";
      case CommWarningKind::CanDuplicateChunk: return "CAN_DUPLICATE_CHUNK";
      case CommWarningKind::CanIncompleteChunk: return "CAN_INCOMPLETE_CHUNK";
      case CommWarningKind::IpcReceiveFailure: return "IPC_RECEIVE_FAILURE";
      case CommWarningKind::IpcLoanFailure: return "IPC_LOAN_FAILURE";
      case CommWarningKind::IpcSendFailure: return "IPC_SEND_FAILURE";
      case CommWarningKind::SerialReadFailure: return "SERIAL_READ_FAILURE";
      case CommWarningKind::Count: break;
    }
    return "UNKNOWN";
  }

  void flush_at(uint64_t now)
  {
    for (size_t i = 0; i < slots_.size(); ++i) {
      const auto kind = static_cast<CommWarningKind>(i);
      auto & slot = slots_[i];
      const uint64_t current = slot.count.load(std::memory_order_relaxed);
      const uint64_t previous = slot.reported.load(std::memory_order_relaxed);
      if (current == previous) continue;

      char context[kContextSize];
      {
        std::lock_guard<std::mutex> lock(slot.context_mutex);
        std::strncpy(context, slot.context, sizeof(context) - 1);
        context[sizeof(context) - 1] = '\0';
      }

      logger()->warn(
        "[CommWarn] {} total={} new={} cmd=0x{:04X} expected={} actual={} context={} last_ns={}",
        name(kind), current, current - previous,
        static_cast<uint32_t>(slot.cmd_id.load(std::memory_order_relaxed)),
        slot.expected.load(std::memory_order_relaxed),
        slot.actual.load(std::memory_order_relaxed), context,
        slot.last_ns.load(std::memory_order_relaxed));

      slot.reported.store(current, std::memory_order_relaxed);
    }
    last_flush_ns_.store(now, std::memory_order_relaxed);
  }

  std::array<Slot, static_cast<size_t>(CommWarningKind::Count)> slots_{};
  std::atomic<uint64_t> total_{0};
  std::atomic<uint64_t> last_flush_ns_{0};
};

inline CommWarnings & comm_warnings()
{
  static CommWarnings instance;
  return instance;
}

}  // namespace info_relay

#endif  // INFO_RELAY__TOOLS__COMM_WARNING_HPP
