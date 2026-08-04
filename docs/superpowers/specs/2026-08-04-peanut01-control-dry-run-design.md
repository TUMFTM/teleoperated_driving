# Peanut01 控制链路 Dry-Run 设计

## 目标

在不连接真实执行器的前提下，验证 G923 方向盘、油门和刹车指令能够从操作端经过 TOD 网络到达车端。首版只验证控制链路，不接收相机、激光雷达、投影或轨迹引导数据。

## 安全边界

- 车辆没有物理急停，因此首版不得包含真实执行器发布代码。
- 禁止发布 `/control/command/control_cmd`、`/cmd_vel`、`/minguo/teleop_override`。
- 禁止打开或写入 SocketCAN 设备。
- 车端唯一新增输出为 `/debug/tod_peanut01/control_cmd`，消息类型沿用 `tod_vehicle_msgs/msg/PrimaryControlCmd`。
- 二级控制命令只在原始 TOD 话题上记录，不转换为车辆命令。

## 架构与数据流

```text
G923
  -> tod_input_devices
  -> tod_command_creation
  -> TOD data network
  -> tod_command_forwarder
  -> tod_safety_gate
  -> tod_peanut01_interface
  -> /debug/tod_peanut01/control_cmd
```

新增 `tod_peanut01_interface`，仅订阅安全门后的主控制命令并生成调试消息。它保留原始 `steering_wheel_angle`、`velocity` 和 `acceleration`，并按 dry-run 参数 `steering_ratio=16.0` 计算：

```text
steering_tire_angle = steering_wheel_angle / 16.0
```

该换算只用于观察和记录，不作为真实车辆标定值。

## 超时处理

- 收到主控制命令后立即发布对应调试消息。
- 连续 `300 ms` 未收到新命令时，发布所有控制字段为零的调试消息。
- 超时后保持内部零状态，不重复高频发布零消息；下一条有效命令到达后恢复输出。
- 节点启动时处于零状态，未收到命令前不生成非零输出。

## 最小启动配置

增加独立的 Peanut01 control-only 启动配置，不替换项目默认配置。

操作端保留：

- `tod_input_devices`
- `tod_command_creation`
- `tod_visual`，仅因为当前 Manager 与可视化程序打包在同一启动文件中
- `tod_communication_interface`
- `tod_state_machine`
- `tod_data_interface`
- `tod_network_monitoring`

车端保留：

- `tod_command_forwarder`
- `tod_safety_gate`
- `tod_topic_monitoring`
- `tod_communication_interface`
- `tod_state_machine`
- `tod_data_interface`
- `tod_network_monitoring`
- `tod_peanut01_interface`

两端关闭 `tod_rtsp`、`tod_lidar`、`tod_projection`、`tod_pure_pursuit`、`tod_trajectory_guidance` 和 `tod_transform`。运行参数使用 `vehicleID=peanut01`、`mode=vehicle`，网络接口继续采用各机器已经验证的接口。

## 记录与验收

记录以下话题：

- `/operator/input_devices/output/joystick`
- `/operator/direct_control/output/primary_control_cmd`
- `/vehicle/network/data/from_operator/primary_control_cmd`
- `/debug/tod_peanut01/control_cmd`
- 原有二级控制命令话题

自动测试覆盖转向换算、字段透传、300 ms 超时归零和超时后恢复。集成验证应满足：

1. 转动方向盘及踩踏板时，四级主控制话题依次出现对应变化。
2. 停止操作或断开操作端后，调试输出在 300 ms 后归零。
3. ROS 图中没有节点发布三类禁止的真实控制话题。
4. Peanut01 接口进程没有打开 CAN 设备，现有车辆业务容器保持不变。

## 不在首版范围

- 真实车辆执行器、CAN 或 Autoware 控制接入
- 车辆动力学参数标定
- 方向盘到前轮转角的正式标定
- 相机、雷达、定位、投影和轨迹控制
- 实车或架空车轮驱动测试
