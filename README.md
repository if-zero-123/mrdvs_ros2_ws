# mrdvs_ros2_ws

## 工具用途

这是一个 ROS2 工作空间，主要包含 `lx_camera_ros` 包，用于接入蓝芯 MRDVS / Lx Camera SDK，构建相机、障碍物、托盘识别、定位和传感器仿真相关节点；同时集成了 `fastlio2` 主里程计包，用于尝试用 MRDVS 的 LiDAR 点云和 IMU 跑 FAST-LIO2。

## 核心逻辑

- `lx_camera_ros` 通过 `ament_cmake` 构建。
- `CMakeLists.txt` 生成自定义 `msg` 和 `srv` 接口，编译 `lx_camera_node`、`lx_localization_node`、`sensor_sim_node`。
- 相机 SDK 头文件和动态库默认来自 `/opt/MRDVS/include` 与 `/opt/MRDVS/lib`。
- `launch/` 提供相机、雷达、障碍物、托盘、定位、建图、传感器仿真等启动入口。
- 根目录脚本提供常用构建和话题查看命令。
- `fastlio2` 来自 `liangheming/FASTLIO2_ROS2` 的主里程计包，本工作空间只接入里程计，不接入 PGO、HBA、localizer 等额外模块。
- `fastlio2` 已从 Livox `CustomMsg` 适配为标准 `sensor_msgs/msg/PointCloud2` 输入，读取 MRDVS LiDAR 模式下的 `x/y/z/intensity/timestamp` 字段，并把点时间换算为 FAST-LIO2 去畸变所需的帧内相对毫秒。
- MRDVS IMU SDK 输出单位已经是 `m/s^2` 和 `rad/s`，因此 `fastlio2` 中不再对线加速度额外乘以 10。

## 代码结构

- `src/lx_camera_ros/`：ROS2 功能包源码。
- `src/fastlio2/`：FAST-LIO2 ROS2 主里程计包，已适配 MRDVS 的 `PointCloud2` 和 `Imu` 话题。
- `src/lx_camera_ros/src/lx_camera/`：相机节点相关实现。
- `src/lx_camera_ros/src/lx_localization/`：定位与传感器仿真相关实现。
- `src/lx_camera_ros/src/utils/`：动态库加载等通用工具。
- `src/lx_camera_ros/msg/`：自定义消息定义。
- `src/lx_camera_ros/srv/`：自定义服务定义。
- `src/lx_camera_ros/launch/`：ROS2 launch 文件。
- `src/lx_camera_ros/rviz/`：RViz 配置。
- `docs/`：SDK 和定位相关说明文档。
- `build/`、`install/`、`log/`：ROS2/colcon 生成目录，不应提交到 Git。

## 使用方法

构建工作空间：

```bash
./build.sh
```

或手动构建：

```bash
rm -rf build install log
colcon build
source install/setup.bash
```

查看常用话题：

```bash
./obstacle.sh
./pallet.sh
./rate.sh
```

录制所有 ROS2 话题到主目录 `bag` 文件夹：

```bash
./record_bag.sh <bag_name>
```

录制输出目录为 `~/bag/<bag_name>`。脚本会拒绝覆盖已经存在的同名 bag，录制时按 `Ctrl+C` 停止。

### 运行 MRDVS + FAST-LIO2

安装 FAST-LIO2 需要的系统依赖：

```bash
sudo apt install libeigen3-dev libpcl-dev libyaml-cpp-dev ros-jazzy-pcl-conversions ros-jazzy-sophus
```

构建相机驱动和 FAST-LIO2：

```bash
colcon build --packages-select lx_camera_ros fastlio2
source install/setup.bash
```

先启动 MRDVS 的 LiDAR 模式。这个 launch 会发布带强度和时间戳字段的 `/lx_camera_node/LxCamera_Cloud`，并发布 `/lx_camera_node/LxCamera_Imu`：

```bash
ros2 launch lx_camera_ros lx_lidar_ros.launch.py enable_rviz:=false
```

确认话题有数据：

```bash
ros2 topic hz /lx_camera_node/LxCamera_Cloud
ros2 topic hz /lx_camera_node/LxCamera_Imu
ros2 topic echo --once --no-arr /lx_camera_node/LxCamera_Cloud
```

再启动 FAST-LIO2：

```bash
ros2 launch fastlio2 mrdvs_lio_launch.py
```

如需同时打开 FAST-LIO2 的 RViz：

```bash
ros2 launch fastlio2 mrdvs_lio_launch.py enable_rviz:=true
```

当前 `src/fastlio2/config/mrdvs.yaml` 中 `r_il` 和 `t_il` 先使用单位阵和零平移，只适合联调启动链路。FAST-LIO2 使用的外参方向是 LiDAR 到 IMU：

```text
p_imu = r_il * p_lidar + t_il
```

实际建图前需要用 SDK `LX_PTR_IMU_EXTRIC_PARAM` 或离线标定结果确认该方向后再填入，否则容易出现漂移、姿态错误或无法初始化。

### 读取 SDK IMU 外参

`lx_camera_ros` 提供一个独立命令行工具，用于通过 SDK 读取 `LX_PTR_IMU_EXTRIC_PARAM`。SDK 注释说明该参数为 12 个 `float`：前 9 个是旋转矩阵，后 3 个是平移向量。工具会原样打印 SDK 数值，同时输出一个便于和 `fastlio2/config/mrdvs.yaml` 对照的 YAML 候选片段；如果 SDK 返回全 0，工具会提示这不是有效旋转矩阵，不能直接作为外参使用。

构建并加载环境：

```bash
colcon build --packages-select lx_camera_ros
source install/setup.bash
```

读取默认设备索引 `0`：

```bash
ros2 run lx_camera_ros read_imu_extrinsic
```

或指定设备索引 / IP：

```bash
ros2 run lx_camera_ros read_imu_extrinsic 0
ros2 run lx_camera_ros read_imu_extrinsic 192.168.1.10
```

注意：SDK 打开设备是独占的。如果 `lx_camera_node` 或其他 SDK 程序正在运行，读取工具可能打不开设备；先停止相机节点，必要时等待几秒让 SDK 心跳释放权限。该工具不会自动转换平移单位，也不会自动写入 FAST-LIO2 配置；需要先确认 SDK 外参方向和单位后再使用。

## Codex 工作规则

当前工作空间的 Codex 用户规则写在 `AGENTS.md`。后续 Codex 在本目录内工作时，应先读取并遵循该文件。

## 更新记录

- 2026-07-06：新增 `read_imu_extrinsic` 工具，通过 SDK 读取 `LX_PTR_IMU_EXTRIC_PARAM`，打印 IMU 外参原始 12 个 float、旋转矩阵、平移向量和 YAML 候选片段，并检测全 0 无效外参，用于后续与标定结果对比。
- 2026-07-06：接入 `liangheming/FASTLIO2_ROS2` 的 `fastlio2` 主里程计包，移除 Livox 消息依赖，适配 MRDVS 的 `PointCloud2` 点云和 IMU 话题，新增 `mrdvs.yaml` 与 `mrdvs_lio_launch.py`。
- 2026-07-06：新增 `record_bag.sh`，支持传入 bag 名称并将所有 ROS2 话题录制到 `~/bag/<bag_name>`。
- 2026-07-06：开启 `lx_camera_ros.launch.py` 的 `LX_BOOL_ENABLE_IMU`，并补充 IMU 加速度与角速度量程默认参数，使 `/lx_camera_node/LxCamera_Imu` 能在重启 launch 后输出数据。
- 2026-07-06：开启 `LX_INT_RGBD_ALIGN_MODE`，使点云使用 RGB 对齐后的颜色；将相机姿态调整为 `roll=-90.0`、`pitch=0.0`、`yaw=-90.0`，把相机光学坐标旋转到 ROS `base_link` 坐标系。
- 2026-07-06：将 `lx_camera_ros.launch.py` 中的 `LX_INT_XYZ_UNIT` 从 `0` 调整为 `1`，使点云按米单位发布，避免 RViz 将毫米坐标当作米坐标显示到远处。
- 2026-07-06：新增 `AGENTS.md`，记录当前工作空间的 Codex 用户规则；新增本 `README.md`，补充项目用途、核心逻辑、结构、使用方法和更新记录；新增 `.gitignore`，避免提交 ROS2 构建产物。
