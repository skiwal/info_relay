# info_relay：串口与导航通信中转进程

整机唯一的串口持有者。本工程只维护一套 iceoryx2 中转系统，不接入 ROS 2；
视觉与导航侧通过 iceoryx2 共享内存发布订阅交换数据，ROS 2 桥接由导航侧自行维护。

---

## 1. 职责边界

| info_relay 负责 | 视觉进程负责 |
| --- | --- |
| 打开/关闭串口，断链重连 | 相机采集与 YOLO 推理 |
| 分帧、CRC、按 cmd_id 号段分派、赛季协议解析 | PnP 解算、跟踪滤波、瞄准决策 |
| 把解析结果发布会到 iceoryx2 | 订阅上行数据、发布云台指令 |
| 订阅视觉瞄准/导航速度指令并打包写串口 | 录像（`tools::Recorder`） |
| 统计链路健康并发布 | 图像调试流（`auto_aim::DebugStream`）、开火判据（依据 `can_shoot`） |

**不做**：任何视觉算法、坐标变换、滤波、图像处理，也不引入 ROS 2 节点或话题。

---

## 2. 主题契约

主题为 `<relay_prefix>/<topic>`，默认前缀 `rm_vision`。载荷全部是定长 POD，
定义见 `info_relay/msg/relay_msgs.hpp`，两侧共用同一份头文件，尺寸由 `static_assert` 固化。

| 主题 | 载荷 | 频率 | 方向 |
| --- | --- | --- | --- |
| `imu` | `Imu` | 500 Hz（有新帧才发） | relay → 视觉 |
| `chassis` | `Chassis` | 100 Hz | relay → 视觉 |
| `robot` | `Robot` | 1 Hz | relay → 视觉 |
| `game` | `Game` | 1 Hz | relay → 视觉 |
| `health` | `Health` | 10 Hz | relay → 任意 |
| `cmd_aim` | `AimCmd` | 视觉帧率 120~150 Hz | 视觉 → relay |
| `cmd_nav` | `NavCmd` | 导航帧率 | 视觉/导航 → relay |

时间基准：所有 `t_ns` 都是本机 `steady_clock` 纳秒计数。同一台机器上可以直接比较，
**禁止在订阅侧重新打时间戳**，否则插值与延迟统计都会失真。

---

## 3. 串口解析的输入与输出

**输入**：`IByteTransport::read(buf, size, timeout_ms)` 的字节流，三种实现：

| 模式 | 来源 |
| --- | --- |
| `real` | 真实串口设备 |
| `fake` | 内置虚拟下位机（按真实协议与频率产帧），或外部 pty |
| `replay` | `serial.bin` 原始字节回放 |

**输出**（解析成功才发布）：

| 输出 | 来源帧 | 失败语义 |
| --- | --- | --- |
| `Imu` | 0x0503 + 0x0500 缓存 | 校验失败丢帧，只在 `Health.crc_errors` 累加 |
| `Chassis` | 0x0502 | 同上 |
| `Robot` | 0x0201 + 0x0202 | 未收到时保持上次值，`valid_mask` 对应位为 0 |
| `Game` | 0x0001 | 未收到时 `valid=0`、`in_battle=0` |
| `Health` | 统计量 | 链路异常时照常发布，`transport_ok=0` |

---

## 4. 订阅端接收约定

| 项 | 约定 |
| --- | --- |
| 缓冲 | 每拍把队列排空，只保留最新一条；IMU 绝不能排队，否则延迟线性累积 |
| 下行 | 视觉侧 `send()` 只写入待发槽，由接收线程下一拍发布，不阻塞主循环 |
| 失链 | 超过 `relay_timeout_ms` 无 IMU 样本 → `link_ok()=false` → `can_shoot()=false`，`shoot` 位被强制清零 |
| 启动顺序 | 两侧可任意顺序启动；relay 未起时视觉侧正常出图，只是不开火 |
| 版本 | `kSchemaVersion` 不符时应拒绝启动（字段顺序变化会让两端静默错位） |

---

## 5. 启动与验证

```bash
# 终端 1：中转进程（虚拟数据源，无需硬件）
./build/info_relay configs/sentry_virtual.yaml

# 终端 2：视觉进程
./build/sentry_multithread configs/sentry_virtual.yaml
```

视觉侧日志出现下面两行即说明链路通：

```
[RelayClient] 已连接主题前缀 rm_vision
[RelayClient] 收到首个 IMU 样本，链路建立
```

依赖自检（不涉及业务逻辑）：

```bash
./build/ipc_selftest     # 期望输出 PASS
```

---

## 6. 延迟实测

`AimCmd.t_ns` 由视觉侧打点，relay 写串口后把 `now - t_ns` 写入 `Health.last_latency_us`。
持续跑一段后观察该字段的 p50 / p99，作为开火时序重新标定的依据。

- 视觉帧周期：120 fps ≈ 8.3 ms；该值应远小于一个帧周期
- 若 p99 接近或超过帧周期，先检查是否有订阅端排队（本实现已按"只取最新"处理）

---

## 7. 依赖

iceoryx2 产物放在 `系统 /usr/local`，由 `cmake/platform.cmake`
按架构选择；产物缺失时顶层 CMake 会跳过 `info_relay` 目标，但视觉侧会因缺少中转客户端
而配置失败——这是刻意的，避免打出一个连不上中转的视觉程序。

获取方式（本机已构建，仓库不纳入版本控制）：

```bash
git clone --depth 1 --branch v0.10.0 https://github.com/eclipse-iceoryx/iceoryx2.git
cmake -S iceoryx2 -B iceoryx2/build -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF
cmake --build iceoryx2/build --parallel          # 需要 cargo
cmake --install iceoryx2/build --prefix /tmp/iox2-install
cp -a /tmp/iox2-install/include /tmp/iox2-install/lib 3rd_part/iceoryx2/amd64/
```

> 注意：cbindgen 生成的头文件可能落在 `target/.../build/iceoryx2-ffi-c-cbindgen/include`，
> 而 CMake 期望 `target/.../iceoryx2-ffi-c-cbindgen/include`；首次构建若报
> `iox2/iceoryx2.h: No such file`，把前者复制到后者再编译即可。
