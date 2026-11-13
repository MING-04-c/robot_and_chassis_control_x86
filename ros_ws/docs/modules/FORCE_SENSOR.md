> 项目自写模块说明已集中到 docs；原位置：ros_ws/src/eli_cs_robot_serial/device/sensor/README.md。文中相对命令路径仍按原模块执行，整机启动以[统一文档入口](../README.md)为准。

# 六维力传感器

USB-RS485 和 TCP-RS485 共用传感器协议与 ROS 接口。厂家确认传感器**重新上电默认 kg 档**；当前默认不再通过两组读数的大小或波动猜测单位。这里配置中的 `kg` 指设备 kgf 档，力矩沿用原协议的 kgf·m 假设，ROS 输出统一为 N、N·m。

## 默认启动与退出

确保传感器刚重新上电，或上一次节点已经正常退出并恢复 kg。启动后自动执行：

1. `AA 55 32 0D 0A`：进入 debug。
2. `AA 55 01 0D 0A`：停止采集，等待 `unit_switch_delay_ms`。
3. 当前已知档位为 kg、目标为 N 时，只发送一次 `AA 55 35 0D 0A`，记录设备档位为 N，等待并丢弃切换前缓存。
4. `zero_before_request: true` 时发送 `AA 55 30 0D 0A` 清零，等待 `zero_wait_ms`（默认 1 秒）。这是原有零偏操作，不是单位比较。
5. 清缓存，发送 `AA 55 02 0D 0A` 开始采集，状态 READY，发布 `/sensor/wrench`。不再等待 100+100 帧或比较 9.80665 倍。

正常退出（Ctrl+C / SIGINT / SIGTERM）时：停止采集，等待 `stop_exit_delay_ms`，若已知当前档位为 N，进入 debug 并发送一次 `35` 切回 kg，再发送 `31` 退出 debug。当前已经是 kg 则不再翻转。正常退出后，同一传感器不重新上电也可以再次启动。

`35` 只有“翻转档位”的含义，不包含目标单位，也没有在现有协议中实现绝对档位读回。READY 依据厂家默认值、已知档位和命令发送成功，不代表通过标准负载验证了量程或精度。

## 配置

`device/sensor/config/sensor_tcp.yaml` 与 `sensor_usb.yaml`：

```yaml
unit: "kg"                        # 首次连接前的已知档位，厂家上电默认 kg
switch_unit_on_startup: true        # 按已知档位对齐目标；已是目标就不重复翻转
restore_kg_on_shutdown: true        # 正常退出恢复 kg
auto_calibrate_unit: false         # 不做两组数据比较
target_unit: "N"                   # 运行时硬件档位
```

以上各项在实际 ROS 参数区中必须处于相同缩进层级。默认 `target_unit: N` 时设备输出 N，软件系数为 1，避免重复乘 9.80665。目标也可选 `kgf`（兼容 `kg`），设备保持 kg 档，软件乘 9.80665，ROS 仍输出 SI。`keep` 保留当前已知档位。不要直接修改参数值来冒充硬件档位切换。

`switch_unit_on_startup: false` 且 `auto_calibrate_unit: false` 才是完全人工声明输入单位的旧方式。旧比较实验代码保留，仅显式设置 `auto_calibrate_unit: true` 才启用；此时不使用默认的上电档位流程。一般使用保持默认配置即可。

## 启动命令

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch eli_cs_robot_serial sensor_tcp.launch.py
```

默认 TCP 地址为 `169.254.199.80:9001`，网口转换器为 TCP Server、节点为 TCP Client。可传 `host:=... port:=...`。
现场已确认转换器串口侧是 **115200、8N1**，不是此前文档写的460800；TCP节点的本机 `baudrate` 不会改变转换器串口侧设置。

USB 入口：

```bash
ros2 launch eli_cs_robot_serial sensor_usb.launch.py
```

查看状态、统一单位数据及绘图：

```bash
ros2 topic echo /sensor/unit_status
ros2 topic echo /sensor/wrench --once
ros2 launch eli_cs_robot_serial sensor_plot.launch.py
```

机械臂导纳、检测及联合任务已经复用这些配置，不要同时启动第二个传感器节点。

## 接收延迟与静止诊断

TCP配置默认 `tcp_no_delay: true`、`tcp_quickack: true`。后者在连接和每次正长度read后请求Linux立即ACK，
因为QUICKACK不是永久状态；不改变网关打包参数、波特率、设备采样率或力保护阈值，也不能保证消除延迟。
不支持/设置失败会报警并继续采集，统计字段显示最近一次设置是否成功。

`timing_report_period_sec: 5.0` 控制诊断发布周期（0关闭），不是采样周期。终端每周期打印 `Sensor timing`，
`/sensor/timing` 同时发布JSON：

| 字段 | 含义 |
|---|---|
| `read_hz` | 电脑每秒正长度read次数，不等于网包数或采样Hz |
| `max_read_gap_ms` / `mean_read_gap_ms` | 相邻正长度read间隔，采用电脑单调时钟 |
| `max_bytes_per_read` | 单次read最大字节数 |
| `max_decoded_frames_per_read` | 一次read中最多解码多少帧，观察成批数据 |
| `wrench_hz` | 节点成功发布的力帧数/窗口时间，不是订阅端测量结果 |
| `since_last_read_ms` | 距最近一次read的时间，检查当前断流 |
| `tcp_quickack_requested` / `tcp_quickack_last_success` | 立即ACK开关及最近一次设置结果 |

当前29字节协议帧没有硬件采样时间戳；`/sensor/wrench`填电脑发布时刻，不能据此证明设备到电脑的端到端延迟。
read可能合并多个网包，单看成批read也不能确定缓存就在网关，必要时结合抓包或USB-RS485直连比较。

先保证机器人静止、没有接触/导纳任务，再只启动一个传感器节点；不要为了测频率启动自动整机工作流：

```bash
ros2 launch eli_cs_robot_serial sensor_tcp.launch.py tcp_quickack:=true
# 另一终端观察，不能同时再启动第二个sensor节点：
ros2 topic echo /sensor/timing
ros2 topic hz /sensor/wrench
```

正常退出后可用 `tcp_quickack:=false` 单独再跑一轮，比较最大read间隔和成批帧数，而不只比较平均Hz。
这会沿用原有的启动单位/清零与正常退出恢复流程；没有新增传感器协议命令。

## 单位服务

正常使用不需要调用单位服务，launch 后自动对齐 N。

```bash
# 必须在机器人没有进行导纳/接触动作时使用。
ros2 service call /sensor/set_unit eli_cs_robot_serial/srv/SetSensorUnit '{unit: kgf}'
ros2 service call /sensor/set_unit eli_cs_robot_serial/srv/SetSensorUnit '{unit: N}'
```

单位已是目标时不翻转；切换中暂停控制用力输出，清理缓存后再恢复。`/sensor/toggle_unit` 翻转当前已知档位，ROS 输出仍为 SI；旧 `/sensor/calibrate_unit` 名称兼容保留，在默认模式中只重新对齐目标，不执行采样比较。单位未知时这些接口拒绝盲目切换。

`/sensor/stop` 停止采集并退出 debug，保留当前已知档位；`/sensor/start` 按目标重新开始，已经在 N 就不会再发 `35`。只有退出节点时默认恢复 kg。`/sensor/zero` 为清零，`/sensor/set_baud` 使用配置的 `baud_code`，`/sensor/write` 为原始 HEX 接口，禁止绕过单位管理直接写单位切换命令。

## 断线与异常退出

TCP/USB 重连不等于传感器重新上电。首次连接失败可以继续重试；但连接成功后再次断开，无法区分“网络短断但仍在 N”和“设备掉电后回到 kg”，所以将档位置为 unknown，重连后不重复发 `35`，暂停 `/sensor/wrench`。重新给传感器上电并重启节点，恢复默认 kg→N 流程。

传感器重新上电后必须同时重启节点。强制杀进程、系统断电、连接已中断时，退出清理可能无法执行，不能保证恢复 kg。这种情况再次使用前应重新给传感器上电。也不要由 其他程序/终端脚本/另一个节点在运行中偷偷改变传感器档位。退出日志会说明是否已发送恢复命令。

## 数据与坐标系

支持 29 字节二进制帧：`AA 55`、命令码 `02/03`、24 字节大端 IEEE754 浮点、`0D 0A`；也支持 debug 的 `channels: v1,v2,v3,v4,v5,v6` ASCII 行。

- `/sensor/wrench`：控制用 SI 数据，仅在单位已知且采集开启后输出。
- `/sensor/unit_status`：保留最新档位、ready、目标档位、换算系数及状态说明。
- `/sensor/raw_values`：六维原始数值，仅供观察，不是控制用力。
- `/sensor/rx`、`/sensor/rx_bytes`：帧级 HEX/字节。
- `/sensor/raw_rx`、`/sensor/raw_rx_bytes`：底层读到的数据块。

`frame_id` 默认 `force_sensor`，数据沿传感器本体 XYZ 轴表达。实际安装存在旋转时必须提供 `tool0 -> force_sensor` 固定 TF，不能只改 frame_id 名称。

绘图 `data_source:=auto` 在没有 SI 输出时显示原始观察值，标题 RAW/UNVERIFIED；收到 SI 输出后切换为 N/N·m。也可选 `data_source:=raw` 或 `verified`。CSV 同时记录数据来源和设备单位。

## 本机模拟验证

```bash
python3 src/eli_cs_robot_serial/device/sensor/test/test_unit_lifecycle.py
```

测试使用隔离 ROS 域与本机假传感器，不连接实际地址，检查一次切换、停止再开始不重复切换、kg 档软件换算、正常退出恢复、再次启动及断线时不猜测单位。不能替代真传感器的协议确认和实际负载验证。
