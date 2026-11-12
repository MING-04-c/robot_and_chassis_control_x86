# 麒麟 TCP 工具说明

当前多轮真机入口及完整报文见 [OLINE流程与协议](OLINE流程与协议.md)，
部署按 [QUICKSTART](QUICKSTART.md) 操作。

唯一工具代码源：`ros_ws/src/eli_cs_robot_serial/device/industrial_pc/host_tools/`。
唯一文档源：`ros_ws/docs/tcp/`。
在仓库根目录执行 `python3 tools/export_industrial_pc.py` 生成独立交付目录
`industrial_pc_kylin_tcp/` 和 ZIP；交付说明由本目录自动复制，不在生成包里改第二遍。

工具只需 Python 3.8+ 标准库，无需 ROS 或相机 SDK。
先在交付目录核对 `config.json`：`robot_host` 是机器人地址，`output_dir` 是麒麟保存位置。
机器人端 `robot_task_oline.yaml` 的 `receiver_ip` 必须是麒麟地址，当前示例为 `169.254.199.250`，
不沿用旧说明里的 `.120` 作为必须值。导出不会覆盖交付目录已有的现场配置。

9010 为命令/状态/坐标与三图保存确认；9002 为 HK，9030 为 RGB-D。
HK 自身线缆协议无落盘 ACK，但 oline 新增的 9010 保存确认覆盖三图真实落盘和校验。
默认无共享密钥，SHA256 仅验证完整性，不是身份认证；如启用 HMAC，两端须同步配置。

`listen_robot.py` 常驻收图和约每 2 秒查询状态，自动确认三图保存，并收逐点测厚结果、电量及异常。
未知/过期电量为 null，不当作 0%；主任务充电状态布局为 `byte_flags`，单独桥默认值可能不同。
心跳超时会提醒并重连，通信恢复不自动发送运动命令；未新增“断网自动回家”功能。

正常 oline 不使用独立 `start_capture.py`，避免抢占相机。每轮必须使用本次新图/深度取点和对应哈希；
不能用 DEMO 点、旧照片坐标或仅改哈希代替真实重新观测。
应答表示受理，完成和停止须看后续状态。

照片按采集时间保存到 `received/年.月.日.时.分/hk/`、同级 `RGB/`，
原始深度保留 uint16；配对、内参、深度比例、哈希见根目录 `.capture_index.json`。
逐点结果按 result_id 去重保存到 `detection_results/`，包含读数和回位状态。

底层历史协议兼容字段见 [TCP通讯协议](TCP通讯协议_麒麟工控机.md)，
旧工具说明由仓库统一归档；旧入口和自动重试描述不覆盖当前 oline 流程。
