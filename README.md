# mrdvs_ros2_ws

## 工具用途

这是 MRDVS 的 ROS 2 工作空间，提供蓝芯 Lx Camera 驱动、定位与传感器仿真节点，以及适配 MRDVS LiDAR 和 IMU 的 FAST-LIO2、FAST-LIVO2 和 PGO 功能。

网页采集控制台已拆分至相邻的独立仓库 `../mrdvs_collector`。该仓库负责网页、鲁班猫热点部署、完整 MCAP 录包和网页显示；本工作空间负责构建并提供它依赖的 ROS 2 驱动 underlay。

## 核心逻辑

- `lx_camera_ros` 通过 `ament_cmake` 构建相机、雷达、障碍物、托盘、定位和传感器仿真节点。
- 相机 SDK 默认来自 `/opt/MRDVS/include` 与 `/opt/MRDVS/lib`。
- `fastlio2` 使用标准 `sensor_msgs/msg/PointCloud2` 和 `Imu`，读取 MRDVS 的 `x/y/z/intensity/timestamp` 点云字段。
- `pgo` 同步 FAST-LIO2 机体系点云和局部里程计，提供在线回环、`map -> lio_local` 修正和优化地图保存服务。
- `fast_livo` 为 FAST-LIVO2 的 ROS 2 移植与 MRDVS 适配版本。

## 代码结构

- `src/lx_camera_ros/`：MRDVS ROS 2 驱动及启动文件。
- `src/fastlio2/`：FAST-LIO2 主里程计包。
- `src/pgo/`、`src/interface/`：在线回环与服务接口。
- `src/fast_livo/`、`src/rpg_vikit/`：FAST-LIVO2 及其视觉工具库。
- `tools/`：传感器 bag 分析与录制辅助工具。
- `build/`、`install/`、`log/`：colcon 生成目录，不提交到 Git。

## 使用方法

构建工作空间：

```bash
./build.sh
```

运行前加载 ROS 2 与本工作空间环境：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## 更新记录

- 2026-08-03：网页采集控制台拆分为本机独立 Git 仓库 `../mrdvs_collector`；本仓库保留 ROS 2 驱动和算法代码。
