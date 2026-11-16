/*
 * iceoryx2 最小联通自检：在单进程内建一对发布者/订阅者，验证本机能否创建服务、
 * 共享内存能否收发、载荷能否按原样往返。
 *
 * 用途：在改动业务代码之前先确认依赖可用（服务创建、/dev/shm、载荷零拷贝），
 *       失败时能明确区分"是框架不可用"还是"是业务代码写错了"。
 *
 * 边界：只验证同一进程内的发布订阅，不涉及跨进程；不写任何文件；
 *       服务名带进程号，避免上次异常退出残留的服务状态影响本次结果。
 */
#include "iox2/iceoryx2.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>
#include <unistd.h>

namespace
{
// 自检载荷：与项目消息契约一样使用固定尺寸 POD，纯值类型，可平凡复制
struct alignas(8) TestPayload
{
  uint64_t seq;    // 递增序号，用于校验顺序与丢样
  uint64_t t_ns;   // 发送时刻，ns；仅用于确认载荷按原样往返
};

constexpr uint64_t kSamples = 200;  // 发送样本数，需大于 0
constexpr auto kCycle = std::chrono::milliseconds(2);  // 每轮发送后的让出间隔

// 功能：取当前单调时钟的纳秒计数
// 输入：无
// 输出：自进程启动以来的纳秒数（steady_clock）
// 备注：只用于填充载荷，不参与判定
uint64_t now_ns()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch())
      .count());
}
}  // namespace

int main()
{
  using namespace iox2;

  // 服务名带进程号：iceoryx2 的服务会残留在共享内存里，固定名字会让重复运行互相干扰
  const std::string service_name = "rm_vision_selftest_" + std::to_string(::getpid());

  auto node = NodeBuilder().create<ServiceType::Ipc>();
  if (!node.has_value()) {
    std::printf("FAIL: 无法创建 iceoryx2 节点\n");
    return 1;
  }

  auto name = ServiceName::create(service_name.c_str());
  if (!name.has_value()) {
    std::printf("FAIL: 服务名非法：%s\n", service_name.c_str());
    return 1;
  }

  auto service =
    node.value().service_builder(name.value()).publish_subscribe<TestPayload>().open_or_create();
  if (!service.has_value()) {
    std::printf("FAIL: 无法创建服务（检查 /dev/shm 是否可写）\n");
    return 1;
  }

  auto publisher = service.value().publisher_builder().create();
  auto subscriber = service.value().subscriber_builder().create();
  if (!publisher.has_value() || !subscriber.has_value()) {
    std::printf("FAIL: 无法创建发布者或订阅者\n");
    return 1;
  }

  uint64_t received = 0;    // 实际收到的样本数
  uint64_t last_seq = 0;    // 上一个收到的序号，用于校验单调递增
  bool ordered = true;      // 序号是否严格递增（1,2,3...）
  bool payload_ok = true;   // 载荷的 t_ns 是否落在合理区间

  for (uint64_t i = 1; i <= kSamples; ++i) {
    auto loaned = publisher.value().loan_uninit();
    if (!loaned.has_value()) {
      std::printf("FAIL: 第 %llu 帧 loan 失败\n", static_cast<unsigned long long>(i));
      return 1;
    }

    const uint64_t t_ns = now_ns();
    auto initialized = loaned.value().write_payload(TestPayload{i, t_ns});
    if (!send(std::move(initialized)).has_value()) {
      std::printf("FAIL: 第 %llu 帧发送失败\n", static_cast<unsigned long long>(i));
      return 1;
    }

    std::this_thread::sleep_for(kCycle);

    // 把当前可取的样本全部取走；receive() 返回 expected<optional<sample>>
    auto taken = subscriber.value().receive();
    while (taken.has_value() && taken.value().has_value()) {
      const auto & payload = taken.value().value().payload();
      if (payload.seq != last_seq + 1) ordered = false;
      if (payload.t_ns > now_ns() + 1000000000ULL) payload_ok = false;
      last_seq = payload.seq;
      ++received;
      taken = subscriber.value().receive();
    }
  }

  std::printf("发送 %llu 帧，收到 %llu 帧\n", static_cast<unsigned long long>(kSamples),
              static_cast<unsigned long long>(received));

  if (received == 0) {
    std::printf("FAIL: 一帧都没收到，发布订阅不通\n");
    return 1;
  }
  if (!ordered) {
    std::printf("FAIL: 序号不连续，存在丢样\n");
    return 1;
  }
  if (!payload_ok) {
    std::printf("FAIL: 载荷内容异常\n");
    return 1;
  }

  std::printf("PASS: iceoryx2 发布订阅联通正常\n");
  return 0;
}
