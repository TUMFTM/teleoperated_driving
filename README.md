# TUM FTM Teleoperated Driving

本项目是 [TUMFTM/teleoperated_driving](https://github.com/TUMFTM/teleoperated_driving) 的 Fork，基于 ROS 2 Humble，面向同一局域网内的操作端与车端远程驾驶实验。

![系统概览](doc/figures/visual_abstract.png)

## 本 Fork 的修改

- 增加 Logitech G923 输入配置和固定回正力工具。
- 将整个 `/dev/input` 挂载到操作端容器，设备编号变化后不需要修改 Compose。
- 操作端和车端网卡可在 `.env` 中分别配置。
- 保留 `virtual.yaml`，没有方向盘时也可以启动并使用虚拟输入设备。
- 保留远端实机验证过的低内存构建、国内镜像源和下载重试配置。

## 环境要求

- Ubuntu 22.04（原生系统或桥接网络的虚拟机）
- Docker Engine 和 Docker Compose V2
- Git、Python 3、`vcstool`
- 操作端和车端位于同一局域网，并能互相 `ping`

安装 `vcstool`：

```bash
sudo apt update
sudo apt install -y git python3-vcstool
```

## 获取和构建

```bash
git clone https://github.com/sl-kai/teleoperated_driving.git
cd teleoperated_driving
chmod +x setup_repos.sh
./setup_repos.sh
docker compose build tod_vehicle tod_operator
```

默认使用单任务低内存构建，适合资源受限的虚拟机，但构建时间会更长。

操作端需要允许容器显示图形界面：

```bash
echo "$DISPLAY"
echo "$XAUTHORITY"
xhost +local:docker
```

请在操作端图形桌面的终端中执行启动命令，不要把临时的 Xauthority 绝对路径写入 `.env`。

## 配置

查看本机 IPv4 地址和网卡名称：

```bash
ip -br -4 addr
```

编辑 `.env`。两台电脑必须使用相同的 `DOCKER_ROS_DOMAIN_ID`；网卡填写各自连接局域网的接口，例如 `ens33`、`enp3s0` 或 `wlp2s0`：

```dotenv
DOCKER_ROS_DOMAIN_ID=7
DOCKER_CYCLONEDDS_CONFIG=unconfigured
MODE=only_sim
VEHICLE_ID=edgar
VEHICLE_NETWORK_INTERFACE=<LAN_INTERFACE>
OPERATOR_NETWORK_INTERFACE=<LAN_INTERFACE>
```

首次实验建议使用 `MODE=only_sim`。连接真实车辆前，需要完成对应车辆接口、安全机制和参数配置。

## 启动

只启动车端：

```bash
docker compose up -d tod_vehicle
```

只启动操作端：

```bash
docker compose up -d tod_operator
```

同一台电脑同时运行两端：

```bash
docker compose up -d tod_vehicle tod_operator
```

查看状态和日志：

```bash
docker compose ps
docker compose logs --tail 100 tod_vehicle tod_operator
```

停止：

```bash
docker compose down
```

## 双机连接

假设操作端地址为 `<OPERATOR_IP>`，车端地址为 `<VEHICLE_IP>`：

1. 两端确认 `ping <PEER_IP>` 正常，并使用相同的 ROS Domain ID。
2. 车端运行 `docker compose up -d tod_vehicle`。
3. 操作端运行 `xhost +local:docker` 和 `docker compose up -d tod_operator`。
4. 在操作端 Manager 中将 `IP Address Vehicle` 填为 `<VEHICLE_IP>`。
5. `IP Address Operator` 列表来自操作端本机网卡；选择 `<OPERATOR_IP>`。没有显示时点击 `Update`。
6. 点击 `Connect`，选择控制模式和输入配置，再点击 `Start`。

车端不需要手工填写操作端地址。Manager 会在连接消息中把所选操作端地址发送给车端。

## 输入设备

没有方向盘时，在 Manager 中选择 `virtual.yaml`。

使用 Logitech G923 时，先确认 Ubuntu 能识别设备：

```bash
lsusb | grep -i logitech
ls -l /dev/input/js* /dev/input/by-id/*G923* 2>/dev/null
```

然后在 Manager 中选择 `logitechg923.yaml`。需要设置固定回正力时执行：

```bash
sudo python3 src/tod_operator_interface/tod_input_devices/tools/set_g923_autocenter.py 30
```

参数范围为 `0` 到 `100`。虚拟机中还需要把 USB 方向盘连接到虚拟机，而不是宿主机。

## 常见问题

- 容器启动后退出：运行 `docker compose logs --tail 200 <SERVICE>` 查看首个错误。
- Manager 一直停在 `Starting`：检查双方 IP、ROS Domain ID、局域网防火墙和网卡名称。
- 操作端没有窗口：重新运行 `xhost +local:docker`，并确认 `DISPLAY` 有值。
- 找不到方向盘：检查虚拟机 USB 连接和 `/dev/input`，设备自检不等于 Ubuntu 已识别。
- 修改源码或新增输入配置后：重新运行 `docker compose build tod_operator`。

## 上游与许可证

原项目由 TUM FTM 发布。本 Fork 保留上游版权、论文引用和 [GNU LGPL v3](LICENSE) 许可证。发布 Fork 时应保留上游来源，并在 Pull Request 中清楚说明自己的修改。

论文：T. Kerbl et al., *TUM Teleoperation: Open Source Software for Remote Driving and Assistance of Automated Vehicles*, 2025, [doi:10.48550/arXiv.2506.13933](https://doi.org/10.48550/arXiv.2506.13933)。
