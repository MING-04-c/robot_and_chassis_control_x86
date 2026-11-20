# 终点返回中途拍照站

独立启动入口，机器人点位与速度全部继承 `robot_task.yaml`：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_task_control robot_task_station_cycle.launch.py
```

只校验、不启动设备：

```bash
ros2 launch robot_task_control robot_task_station_cycle.launch.py validate_only:=true
```

配置入口是 `config/robot_task_station_cycle.yaml`。原 local/online 入口不启用此功能。
页面和机器人均需更新；TCP命令新增 `action: return_station`。

1. 页面启动任务，前进跳过中途横线，到真正终点停稳，展开机械臂到拍照位等待。
2. 终点等待时点击“返回中途拍照站”：按中间点2、中间点1、过渡位2、过渡位1、行车位收臂，再倒车。
3. 在倒车遇到的最近一条确认的中途横线停车，停稳后经同一套去程点位展开到同一拍照位。
4. 中途等待时点击“检测”：只拍HK、RGB、深度三图，三图保存校验确认后自动收臂并前进。
5. 跳过中途出发横线，前进到终点，展开到拍照位，自动拍照和传输，等待本轮XYZ。
6. 收到本轮有效XYZ后执行测厚、确认回拍照原位，按完整回程点位收臂，再倒车回充电桩。

中途站等待时也可点击“返回充电”：取消前进终点分支，收臂后沿剩余返程路线回充。
拍照传输、测厚接触及底盘行驶期间不会受理并发换站命令。

前进、倒车使用同一进程中已锚定的标准路线。换向前需要禁用循迹、停稳、确认到站横线及新鲜里程计。
中途重新前进的进度由本轮倒车弧长转换，不能以中途位置重新锚定充电起点。
当前支持最近的一条中途站，不支持任意编号选站；倒车到路线终点仍未确认中途横线时停车报错，不展开机械臂。
中途站在相机中必须能被现有横线分类器稳定识别为 intermediate；相邻站过近或横线类型无法确认时保持停车。
