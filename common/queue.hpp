#ifndef INFO_RELAY__COMMON__QUEUE_HPP
#define INFO_RELAY__COMMON__QUEUE_HPP

/*
 * 线程安全队列：有界、带溢出策略，供串口接收线程与业务线程之间传递数据。
 *
 * 数据流：生产者 push → 内部 deque → 消费者 pop / try_pop / try_pop_for。
 *
 * 依赖与边界：只用标准库；不做内存池、不做无锁化，临界区只包住入队出队本身。
 *
 * 使用注意：满队列的处理方式由 Overflow 决定，必须按调用点的业务含义选择：
 *           Block      生产者可以等（如命令行工具），不丢数据但会阻塞；
 *           DropOldest 只关心最新值（如姿态插值队列），丢最旧的样本；
 *           DropNewest 宁可丢新数据也不让生产者等待（如串口上报），是默认策略。
 */

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <utility>

namespace info_relay
{

// 队列满时的处理策略
enum class Overflow
{
  Block,       // 阻塞生产者直到有空位
  DropOldest,  // 丢弃队首最旧的元素以腾出位置
  DropNewest,  // 丢弃本次入队的元素
};

template <typename T>
class Queue
{
public:
  // 功能：构造队列
  // 输入：
  //   capacity  容量上限，必须大于 0
  //   policy    满队列策略
  // 输出：无
  // 备注：容量在生命周期内不变，避免运行期抖动
  explicit Queue(size_t capacity, Overflow policy = Overflow::DropNewest)
  : capacity_(capacity == 0 ? 1 : capacity), policy_(policy)
  {
  }

  // 功能：入队（拷贝）
  // 输入：
  //   value  待入队元素
  // 输出：无
  // 备注：Block 策略下可能阻塞；其余策略下立即返回
  void push(const T & value) { push_impl(value); }

  // 功能：入队（移动）
  // 输入：
  //   value  待入队元素，会被移走
  // 输出：无
  // 备注：大对象（帧缓冲、检测结果）应走这里，省一次拷贝
  void push(T && value) { push_impl(std::move(value)); }

  // 功能：阻塞取队首
  // 输入：
  //   value  输出参数，成功后写入取到的元素
  // 输出：无
  // 备注：队列空时一直等待；只应在确定会有数据的消费者线程调用
  void pop(T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    not_empty_.wait(lock, [this] { return !queue_.empty(); });
    value = std::move(queue_.front());
    queue_.pop_front();
    not_full_.notify_one();
  }

  // 功能：阻塞取队首并返回
  // 输入：无
  // 输出：取到的元素
  // 备注：语义同 pop(T &)，供 `auto x = q.pop();` 使用
  T pop()
  {
    T value{};
    pop(value);
    return value;
  }

  // 功能：非阻塞取队首
  // 输入：
  //   value  输出参数
  // 输出：true 表示取到元素，false 表示队列为空
  // 备注：不加等待，适合"每拍排空队列"的场景
  bool try_pop(T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (queue_.empty()) return false;
    value = std::move(queue_.front());
    queue_.pop_front();
    not_full_.notify_one();
    return true;
  }

  // 功能：带超时取队首
  // 输入：
  //   value    输出参数
  //   timeout  最长等待时长
  // 输出：true 表示取到元素，false 表示超时
  // 备注：超时后返回 false，调用方可借此轮询退出标志
  template <typename Rep, typename Period>
  bool try_pop_for(T & value, const std::chrono::duration<Rep, Period> & timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!not_empty_.wait_for(lock, timeout, [this] { return !queue_.empty(); })) return false;
    value = std::move(queue_.front());
    queue_.pop_front();
    not_full_.notify_one();
    return true;
  }

  // 功能：取队尾元素（最近入队）但不弹出
  // 输入：
  //   value  输出参数
  // 输出：true 表示取到，false 表示队列为空
  // 备注：供"只看最新一条"的场景使用，例如导航速度去重
  bool back(T & value) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) return false;
    value = queue_.back();
    return true;
  }

  // 功能：判断队列是否为空
  // 输入：无
  // 输出：true 表示为空
  // 备注：返回值只代表调用瞬间的状态，多线程下不可作为后续操作的前提
  bool empty() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  // 功能：取当前元素个数
  // 输入：无
  // 输出：元素个数
  // 备注：同样只代表调用瞬间
  size_t size() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

  // 功能：清空队列
  // 输入：无
  // 输出：无
  // 备注：会唤醒因满队而阻塞的生产者
  void clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.clear();
    not_full_.notify_all();
  }

private:
  // 功能：统一的入队实现
  // 输入：
  //   value  待入队元素，支持拷贝与移动两种绑定
  // 输出：无
  // 备注：按策略处理满队；Block 用条件变量等待，其余两种直接返回或丢元素
  template <typename U>
  void push_impl(U && value)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    if (queue_.size() >= capacity_) {
      switch (policy_) {
        case Overflow::Block:
          not_full_.wait(lock, [this] { return queue_.size() < capacity_; });
          break;
        case Overflow::DropOldest:
          queue_.pop_front();
          break;
        case Overflow::DropNewest:
          return;
      }
    }

    queue_.push_back(std::forward<U>(value));
    not_empty_.notify_one();
  }

  std::deque<T> queue_;                  // 元素容器，front 为最早入队
  size_t capacity_;                      // 容量上限，构造期确定
  Overflow policy_;                      // 满队策略
  mutable std::mutex mutex_;             // 保护 queue_ 与两个条件变量
  std::condition_variable not_empty_;    // 队列非空通知（消费者等待）
  std::condition_variable not_full_;     // 队列有空位通知（Block 策略的生产者等待）
};

}  // namespace info_relay

#endif  // INFO_RELAY__COMMON__QUEUE_HPP
