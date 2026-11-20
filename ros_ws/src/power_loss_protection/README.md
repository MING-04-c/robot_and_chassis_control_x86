# Power loss protection

MIDUPS 智能模式正式失电保护，Ubuntu 24.04 / ROS 2 Jazzy。
现在和以后部署均使用同一执行文件 `run_power_loss_protection.sh`。
Python 负责监控和 ROS 停止请求，systemd 负责管理 ROS 进程及正常关机。
监控只用 Python 标准库，不依赖互联网或 ROS 存活。
本目录保留 COLCON_IGNORE，不需要 colcon 编译本模块。

## 正式执行

保存文件并退出其他 UPS 监控，在本目录执行：

```bash
sudo ./run_power_loss_protection.sh
```

等价命令为 `sudo python3 power_loss_protection.py run --config config.json`。
该命令真实执行：确认失电后请求本机 ROS 停止，再正常关闭本机。
确认后即使恢复主电也继续关机。主电正常时持续监控；确认前 Ctrl+C 可结束前台程序。
通信持续失效默认1.5秒也会执行关机，包括启动后一直没有有效回复的情况。
当前只提供源码和配置，没有自动安装/启用系统服务，也没有自动执行关机。

`config.json` 是正式配置，mode=armed，网关169.254.199.90:502、RTU透传、从机地址1。
当前 `computer_power_source=external`，表示电脑使用自己的独立电源，UPS只提供被监测信号。
这不会改变失电判定、失联保护或停车/关机流程，只表示本机不依赖该UPS的储能，因此不检查本机UPS续航预算。
**电脑以后接到UPS输出时，改为 computer_power_source=ups，并填写实测备用时间。执行文件不变。**
程序不能自动判断实际接线，必须按现场供电关系配置。

## 保护流程

1. 每500ms读取0x0000～0x0008，校验CRC/MBAP、地址、功能码、长度及状态。
2. BACKUP(bit1)连续三次确认后锁定关机；错误或恢复MAIN打断连续计数。
3. 持续无有效通信、明确待机/断电等待/故障也触发保护。
4. 保存关机锁，撤销受管理ROS服务的启动许可。
5. 限时并发请求任务停止、检测/姿态/轨迹取消、底盘STOP、循迹/速度输出/导纳禁用、保护恢复取消及dashboard stop。
6. 停止robot-control.service。SIGINT通知launch清理，超时后systemd清理服务剩余进程。
7. ROS接口缺失或退出超时不阻止后续systemctl poweroff，由OS正常停止其他服务并同步文件系统。

电压采用V，电流采用A，SOC为百分比；不把AC_OK/EOC普通事件视为故障，不用SOC估算剩余秒数。
不自动回零、回程、回桩、解锁、释放抱闸或恢复任务。
同一开机内服务重启，若已有关机锁，会继续关机；下一次开机保留旧锁并阻止受管理ROS自动重启。
ROS停止回复不代表物理停车已确认，目前没有依据底盘/关节速度验证静止，不替代硬件急停。

## ROS管理范围

停止助手调用同一ROS_DOMAIN_ID中的现有接口，生产launch和子进程应统一由robot-control.service管理。
当前未安装服务时，直接运行会尝试ROS接口，服务不存在会记录错误，然后仍正常关闭系统。
手动在其他终端启动的ROS最终也会在OS关机时关闭，但不保证此前收到本模块SIGINT。
当前只修改新目录，原任务/TCP入口没有集成独立电源锁，停止期间仍有并发运动请求窗口。
启动锁阻止受管理服务重启，不等于封锁所有ROS/TCP请求。
不发布/robot_control/safety_inhibit，避免与已有保护恢复器相互覆盖。

## 开机服务安装

先核对config.json的workspace、robot_user、ros_domain_id、launch_command，以及unit的User、WorkingDirectory和TimeoutStopSec。
示例launch为local主任务auto_start=false；按现场参数调整，不重复启动驱动。
stop helper与受管理launch使用相同ROS_DOMAIN_ID和LOCALHOST发现范围。
若有自定义DDS/RMW配置，需要在ros_environment中为两者配置一致环境。

```bash
sudo install -d /opt/power-loss-protection /etc/power-loss-protection
sudo install -m 644 modbus.py power_loss_protection.py ros_stop.py /opt/power-loss-protection/
sudo install -m 755 run_power_loss_protection.sh /opt/power-loss-protection/
sudo install -m 644 config.json /etc/power-loss-protection/config.json
sudo install -m 644 systemd/power-loss-protection.service systemd/robot-control.service /etc/systemd/system/
sudo systemctl daemon-reload
```

退出前台监控，确认主电和通信正常后实际启用保护：

```bash
sudo systemctl enable --now power-loss-protection.service
journalctl -u power-loss-protection.service -f
```

机器人不自动启动，近期MAIN正常且没有关机锁时，操作员执行：

```bash
sudo systemctl start robot-control.service
journalctl -u robot-control.service -f
```

停用保护使用 `sudo systemctl stop power-loss-protection.service`。
因Requires依赖，停用保护也会停止受管理ROS。不能同时运行前台和系统保护进程。

## UPS带载预算

computer_power_source=ups时，必须填写measured_min_backup_seconds。
检查：备用时间 > 检测预算 + ROS停止 + 进程停止 + 关机请求最多3秒 + OS关机 + 余量。
默认预算要求超过约18秒。os_shutdown_budget_seconds的5秒是初始配置，不是实际关机保证。
ros_stop_seconds和process_stop_seconds分别默认2秒和3秒；后者同时对应unit的TimeoutStopSec。
reserve_seconds默认2秒。不要捏造备用时间；测量不足应调整已验证时序、增加储能或降低负载。
手册标称25V/2.5F，50W负载可能仅十余秒备用。网关与必要网络设备也须保持供电。
配置校验不能证明实际关机耗时，仍须完成真实满载验收。

## 只读检查与维护

`python3 power_loss_protection.py probe`只读取UPS，不关机；`check`只验证配置。
现场已确认rtu_over_tcp成功、modbus_tcp超时，正式运行不自动切换协议。
不要让多个客户端同时轮询RS485。当前采用短连接，早先间歇超时的根因尚未确认。
电压字段vmain_v/vcap_v/vout_v，电流iout_a。
只读观察需要显式选示例配置：

```bash
python3 power_loss_protection.py monitor --config config.example.json
```

默认配置已经改为正式config.json，直接sudo运行monitor同样启用真实保护。
依据仅为产品手册与V3.1.1协议；两者0x0106关断值相反，因此不写UPS寄存器。
不使用HMI或Windows Agent的关闭流程。

重启后确认主电稳定、机器人停止，再解除本模块历史启动锁：

```bash
sudo systemctl stop robot-control.service
sudo rm -- /var/lib/power-loss-protection/shutdown.json
sudo systemctl start robot-control.service
```

仅删除上述确切文件，它不是机械臂原有安全锁，不用于解除报警。
来电自动开机取决于UPS输出周期和BIOS，目前尚未闭环验证。
开发回归：`python3 -m unittest discover -s test -v`，模拟校验不执行真实停车或关机。
