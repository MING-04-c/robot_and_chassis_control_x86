> 历史/兼容资料：保留原文供追溯，其中旧 launch、点位、力阈值和 IP 不代表现行配置。不要直接照旧命令启动；当前操作以[统一文档入口](../README.md)为准。

# 工控机TCP通讯

新增远程多轮入口：`ros2 launch robot_task_control robot_task_oline.launch.py`。
新版麒麟监听器确认三图保存，使用start/detect/go_home驱动会话；电量及异常沿9010状态回传。
详见 [OLINE流程与协议](../tcp/OLINE流程与协议.md)，下文旧任务入口仅供历史参考。

已增加9010应用握手、心跳及双端超时/恢复提示；部署时同步更新外部工具ZIP，
工控机运行listen_robot.py产生心跳。默认2秒轮询、10秒超时，超时持续重连并提示人工检查，
恢复不自动发送启动/继续命令。此功能不新增运动自动停机策略。
ROS在线状态见 `/industrial_pc/heartbeat`，YAML参数heartbeat_timeout配置机器人端超时。
详细字段、兼容开关和照片通道确认范围见host_tools/README.md。

默认无密钥；Pi169.254.199.100，麒麟169.254.199.120。9010用于命令/状态/检测点，9002独立传HK，9030独立传RGB-D。保留SHA256、长度校验、去重及断线重连。

完整运动远程入口为robot_task_control的line_follow_arm_test.launch.py remote_control:=true；只测相机为camera_capture_remote.launch.py。二者已包含本节点，不重复启动。单独桥接其他已加载工作流时，可用 `ros2 launch eli_cs_robot_serial industrial_pc.launch.py`。默认start_hk_transfer=false，让末尾采集流程按需占用9002。

麒麟工具在host_tools/，通过项目tools/export_industrial_pc.py导出。先listen_robot.py，另一个终端start_task.py或start_capture.py；检测点send_points.py。当前无需industrial_pc.key。旧签名模式只在两端明确设置use_shared_key=true并指定key_file时使用。

照片保存为 `年.月.日.时.分/hk/` 和 `年.月.日.时.分/RGB/`，例如 `2026.10.30.20.29/`。HK放海康JPEG，RGB放彩色/深度PNG，文件名以相同的时分秒毫秒前缀配对；原始16位深度不转换为8位，参数和深度比例集中保存在隐藏索引.capture_index.json。传输内部仍校验既有V5.4清单，临时ZIP不重复长期保存。

命令固定桥接/task_controller/start、/stop、/capture及/line_follow/resume。采集与运动互斥，同ID请求持久去重。收到坐标只发布/industrial_pc/camera_points，演示点为/industrial_pc/demo_camera_points，不自动执行触碰。

详细启动步骤见host_tools/QUICKSTART.md，线缆字段和检测点示例见项目根目录TCP通讯协议_麒麟工控机.md。

## TCP通信与额外状态汇总开关

`industrial_pc.launch.py enable_status_monitor:=false` 保留9010 TCP监听、任务开始/停止/继续、
基础任务查询和原有结果通道，关闭正常/异常汇总、相关日志订阅与汇总定时器。
基础状态回包中的`status_monitor_enabled`为false，`normal_status`和`abnormal_status`为null；
单独查询normal/abnormal会明确返回未启用。此开关不关闭设备运动保护。
默认值为true，已有启动方式保持原行为。
