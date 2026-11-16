#ifndef INFO_RELAY__COMMON__CLOCK_HPP
#define INFO_RELAY__COMMON__CLOCK_HPP

/*
 * 会话时钟：以进程启动时刻为零点，提供单调递增的纳秒时间轴。
 *
 * 数据流：串口字节录制记录 t_ns → 回放时按 t_ns 还原节奏；
 *         时间戳只在本进程内有意义，跨进程用 steady_clock 绝对纳秒（见 msg 契约）。
 *
 * 依赖与边界：纯头文件、纯标准库；单例只保存一个起点，无锁。
 *
 * 使用注意：start_time() 在首次调用时固定，之后不再变化；
 *           不要把本时间轴直接当作跨进程时间戳，跨进程一律用 steady_clock 绝对时间。
 */

#include <chrono>
#include <cstdint>

namespace info_relay
{

using SteadyClock = std::chrono::steady_clock;  // 单调时钟，不受系统时间调整影响
using TimePoint = SteadyClock::time_point;      // 时间点别名

class SessionClock
{
public:
  // 功能：取单例
  // 输入：无
  // 输出：进程内唯一实例
  // 备注：首次调用时把当前时刻记为起点
  static SessionClock & instance()
  {
    static SessionClock clock;
    return clock;
  }

  // 功能：取会话起点
  // 输入：无
  // 输出：进程启动后第一次访问该函数的时间点
  // 备注：作为相对时间戳的零点
  static TimePoint start_time() { return instance().start_; }

  // 功能：取相对会话起点的纳秒数
  // 输入：无
  // 输出：自 start_time() 起的纳秒数，单调递增
  // 备注：用于录制与回放的时间轴，不是跨进程时间戳
  static uint64_t now_rel_ns()
  {
    return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(SteadyClock::now() - start_time())
        .count());
  }

  // 功能：把相对纳秒还原成时间点
  // 输入：
  //   ns  相对会话起点的纳秒数
  // 输出：对应的 steady_clock 时间点
  // 备注：回放器用它把录制时间轴映射回当前时钟
  static TimePoint from_rel_ns(uint64_t ns)
  {
    return start_time() + std::chrono::nanoseconds(ns);
  }

private:
  SessionClock() : start_(SteadyClock::now()) {}

  TimePoint start_;  // 会话起点，构造时固定
};

}  // namespace info_relay

#endif  // INFO_RELAY__COMMON__CLOCK_HPP
