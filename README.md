# TUM FTM Teleoperated Driving Software

This is the repository of the TUM FTM Teleoperated Driving Software Stack. The stack is ROS2-based and tested on Ubuntu 22.04 with ROS Humble, only.

![Alt](doc/figures/visual_abstract.png "overview")

The software stack is grouped thematically in the following packages:

- tod_common
- tod_direct_control
- tod_launch
- tod_logging
- tod_monitoring
- tod_msgs
- tod_network
- tod_operator_interface
- tod_perception
- tod_safety
- tod_state_machine
- tod_trajectory_guidance
- tod_vehicle_interface

A video, showcasing the software on three different vehicle systems will be available soon.

## System Architecture

The system architecture is depicted in the following graphic. The color of the packages corresponds to the grouping of the packages in the respective sub-repositories. A more detailed overview of the architecture can be found on the wiki page.

![Alt](doc/figures/tod_architecture.png "system architecture")

## Getting Started

[Getting Started](doc/getting_started.md). Further information about the docker workflow used for development and deployment can be found under [Docker Workflow](doc/docker_workflow.md)

## GHCR 镜像部署

本项目发布两个独立运行镜像，每个镜像同时支持 `linux/amd64` 和
`linux/arm64`，Docker 会自动选择当前机器对应的架构：

```text
ghcr.io/sl-kai/teleoperated-driving-operator
ghcr.io/sl-kai/teleoperated-driving-vehicle
```

首次部署时克隆 `ros2` 分支，根据实际机器修改 `.env` 和 Compose 中的
网卡、DDS、显示及设备配置，然后拉取镜像：

```bash
git clone -b ros2 https://github.com/sl-kai/teleoperated_driving.git
cd teleoperated_driving
docker compose pull tod_operator tod_vehicle
```

在车端启动或停止：

```bash
docker compose up -d tod_vehicle
docker compose stop tod_vehicle
```

在操作端启动或停止：

```bash
docker compose up -d tod_operator
docker compose stop tod_operator
```

`.env` 默认使用 `edge` 标签。正式部署建议改为固定版本，例如：

```env
DOCKER_TAG=1.0.0
```

推送到 `ros2` 分支会构建 `edge`；推送 `v1.0.0` 格式的 Git 标签会构建
对应版本和 `latest`。镜像由 GitHub Actions 自动发布到 GHCR。

## Publication

Kerbl, Tobias, David Brecht, Nils Gehrke, Nijinshan Karunainayagam, Niklas Krauss, Florian Pfab, Richard Taupitz, Ines Trautmannsheimer, Xiyan Su, Maria-Magdalena Wolf and Frank Diermeyer. “TUM Teleoperation: Open Source Software for Remote Driving and Assistance of Automated Vehicles.” (2025), doi: https://doi.org/10.48550/arXiv.2506.13933.
