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
- `src/fast_livo/`：FAST-LIVO2 ROS2 Humble 移植版，当前分支已做 MRDVS 初始适配。
- `src/rpg_vikit/`：FAST-LIVO2 使用的 ROS2 vikit 相机模型和视觉工具库。
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

### 录制 MRDVS + FAST-LIVO2 排查数据包

如果要排查“轻微运动正常、稍微剧烈抖动后漂移”的问题，建议录一个包含原始传感器和 FAST-LIVO2 输出的定向 bag，不必录全量话题。

先启动 MRDVS + FAST-LIVO2：

```bash
source install/setup.bash
ros2 launch fast_livo mrdvs_full_launch.py camera_ip:=192.168.100.82 use_rviz:=True
```

另开一个终端开始录制：

```bash
source install/setup.bash
mkdir -p ~/bag
ros2 bag record -o ~/bag/mrdvs_livo_debug_$(date +%Y%m%d_%H%M%S) \
  /lx_camera_node/LxCamera_Cloud \
  /lx_camera_node/LxCamera_Imu \
  /lx_camera_node/LxCamera_Rgb \
  /tf /tf_static \
  /cloud_registered \
  /aft_mapped_to_init \
  /path \
  /LIVO2/imu_propagate
```

推荐动作流程：

```text
0-10s：设备完全静止，让 IMU 重力和 bias 初始化。
10-25s：慢速平移，前后左右各移动一点，幅度 0.3-0.8m。
25-40s：慢速旋转，分别做 yaw、pitch、roll，小角度即可。
40-55s：轻微抖动，保持画面里有墙面、桌面、纹理物体。
55-70s：做会触发漂移的剧烈抖动，不要遮挡 RGB/ToF，不要对着纯白墙、玻璃或近距离空白区域。
70-80s：重新静止，观察位姿是否还能稳定。
```

录完后用离线工具分析点云点级微秒时间戳、cloud header、RGB 和 IMU 的时间关系：

```bash
source install/setup.bash
tools/analyze_mrdvs_bag.py ~/bag/<bag_name>
```

如果 bag 很大，可以先抽样分析：

```bash
tools/analyze_mrdvs_bag.py ~/bag/<bag_name> --max-clouds 300 --point-stride 4
```

重点看输出中的 `first point - cloud header`、`min point - cloud header`、`point timestamp span`、`image - cloud_header` 和 `image - point_start`。如果点级微秒时间和 header 差了几十毫秒，说明当前点云去畸变的时间基准要改；如果图像和点云长期差几十到上百毫秒，则优先调 `time_offset.img_time_offset`。

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

推荐直接启动 MRDVS + FAST-LIO2 一体 launch。它会先用固定 IP `192.168.100.82` 启动 MRDVS 的 LiDAR 模式，驱动自身 RViz 保持关闭，等待默认 3 秒后再启动 FAST-LIO2，默认打开 FAST-LIO2 的 RViz，并默认使用厂家结构设计外参配置 `mrdvs.yaml`：

```bash
ros2 launch fastlio2 mrdvs_full_launch.py
```

如需调整固定 IP、等待时间或关闭 RViz：

```bash
ros2 launch fastlio2 mrdvs_full_launch.py camera_ip:=192.168.100.82 fastlio_delay:=3.0 enable_rviz:=false
```

如果后续想临时尝试更细的地图细节，可以手动切到精细参数，同时保持厂家结构外参对应的静态 TF：

```bash
ros2 launch fastlio2 mrdvs_full_launch.py config_file:=mrdvs_refined.yaml
```

分步调试时，可以先单独启动 MRDVS 驱动。原始驱动里有两种常用点云启动方式：

彩色点云 / RGBD 对齐显示模式：

```bash
ros2 launch lx_camera_ros lx_camera_ros.launch.py enable_rviz:=true
```

该模式使用 `is_xyz=1` 和 `LX_INT_RGBD_ALIGN_MODE=1`，发布 `PointXYZRGB` 类型的 `/lx_camera_node/LxCamera_Cloud`，适合查看 RGB 着色点云效果；默认没有 FAST-LIO2/FAST-LIVO2 运动补偿需要的点级 `timestamp` 字段。

SLAM 点云 / 强度和时间戳模式：

```bash
ros2 launch lx_camera_ros lx_lidar_ros.launch.py ip:=192.168.100.82 enable_rviz:=true
```

该模式默认固定连接 `192.168.100.82`，不会先枚举设备；使用 `is_xyz=2` 和 `LX_PTR_XYZIRT_DATA`，发布带 `intensity/timestamp/row_pos/col_pos` 字段的 `/lx_camera_node/LxCamera_Cloud`，并发布 `/lx_camera_node/LxCamera_Imu`，适合 FAST-LIO2 / FAST-LIVO2。

如需只启动 LiDAR 模式但不打开驱动 RViz：

```bash
ros2 launch lx_camera_ros lx_lidar_ros.launch.py ip:=192.168.100.82 enable_rviz:=false
```

如需恢复按索引/枚举方式打开设备，可传入短索引值，例如 `ip:=0`；固定 IP 模式下驱动会跳过 `DcGetDeviceList()` 枚举等待，直接调用 SDK 按 IP 打开设备。

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

`mrdvs_lio_launch.py` 默认使用稳定基线配置 `mrdvs.yaml`。如果需要临时尝试稍微精细一点的配置，可以手动切换到 `mrdvs_refined.yaml`：

```bash
ros2 launch fastlio2 mrdvs_lio_launch.py config_file:=mrdvs_refined.yaml enable_rviz:=true
```

`mrdvs_refined.yaml` 相比稳定基线 `mrdvs.yaml` 保留更多点云细节，主要调整为 `lidar_filter_num=2`、`lidar_min_range=0.35`、`lidar_max_range=20.0`、`scan_resolution=0.10`、`map_resolution=0.20`、`imu_init_num=60`、`lidar_cov_inv=800.0`。该配置更吃 CPU；如果现场移动时卡顿或建图延迟，优先使用默认 `mrdvs.yaml`。

FAST-LIO2 的 RViz 配置以 `map` 为 Fixed Frame。当前推荐 TF 树为：

```text
map -> mrdvs_imu -> mrdvs_tof
```

其中 `map -> mrdvs_imu` 由 FAST-LIO2 根据里程计结果动态发布；`mrdvs_imu -> mrdvs_tof` 由 `mrdvs_lio_launch.py` 根据厂家结构外参发布为静态 TF。`lx_lidar_ros.launch.py` 在 LiDAR 模式下关闭驱动自带的 `base_link -> mrdvs_tof` TF，避免 `mrdvs_tof` 同时挂到两个父坐标系下面；如果 RGB 流开启，驱动仍会发布 `mrdvs_tof -> mrdvs_rgb`。普通 `lx_camera_ros.launch.py` 仍保留相机驱动原有 TF，用于单独查看相机/RGBD 数据。

当前 `src/fastlio2/config/mrdvs.yaml` 中 `r_il` 和 `t_il` 已填入厂家提供的结构设计外参 `imu_lidar_ext`。厂家参数说明为 `xyz-ypr`、以 IMU 为准、单位为米和度：

```text
imu_lidar_ext = [0.014569, -0.002738, 0.022567, 0, 0, 0]
```

由于 `ypr` 全为 0 度，`r_il` 使用单位阵；平移直接写入 `t_il`。FAST-LIO2 使用的外参方向是 LiDAR 到 IMU：

```text
p_imu = r_il * p_lidar + t_il
```

后续如果通过离线标定得到更准确结果，可以再用标定结果替换该结构设计外参；如果确认厂家定义的方向不是 LiDAR 在 IMU 坐标系下的位姿，则需要先做方向转换后再填入，否则容易出现漂移、姿态错误或无法初始化。

`src/fastlio2/config/mrdvs_lidar_imu_init.yaml` 也已回退为厂家结构设计外参，保留该文件只是为了兼容之前的启动参数和测试记录：

```yaml
imu_time_offset: 0.0
r_il: [1.0, 0.0, 0.0,
       0.0, 1.0, 0.0,
       0.0, 0.0, 1.0]
t_il: [0.014569, -0.002738, 0.022567]
```

此前 LiDAR_IMU_Init refinement 输出的平移达到约 `[-0.38cm, -12.18cm, -18.90cm]`，和当前设备物理结构不符，因此不再作为 FAST-LIO2/FAST-LIVO2 默认外参。FAST-LIO2 仍保留 `imu_time_offset` 字段和 `r_il` 正交化逻辑，方便后续拿到可信标定结果后再测试。

当前 FAST-LIO2 和 FAST-LIVO2 的 MRDVS 配置已写入本机 Allan 标定得到的 IMU 噪声参数，采用标定文件中的 `avg-axis`：

| 标定项 | 数值 | 单位 | FAST-LIO2 字段 | FAST-LIVO2 字段 |
| --- | ---: | --- | --- | --- |
| `acc_n` | `2.0348936872780068e-02` | `m/s^2` | `na` | `acc_cov` |
| `gyr_n` | `2.3162072810468197e-03` | `rad/s` | `ng` | `gyr_cov` |
| `acc_w` | `4.5452036338894608e-04` | `m/s^2` | `nba` | `b_acc_cov` |
| `gyr_w` | `2.3553527673141791e-05` | `rad/s` | `nbg` | `b_gyr_cov` |

这些值已同步写入 `src/fastlio2/config/mrdvs.yaml`、`src/fastlio2/config/mrdvs_refined.yaml` 和 `src/fast_livo/config/mrdvs.yaml`。FAST-LIVO2 源码也已改为读取 `b_acc_cov`、`b_gyr_cov` 配置项，不再使用写死的默认值。

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

### FAST-LIVO2 标定参数

`lx_camera_ros` 还提供 `read_fastlivo_calib` 工具，用于一次性读取 FAST-LIVO2 初始接入时需要核对的 RGB 内参、ToF/RGB 外参和 ToF/IMU 外参：

```bash
colcon build --packages-select lx_camera_ros
source install/setup.bash
ros2 run lx_camera_ros read_fastlivo_calib 192.168.100.82
```

当前设备 `camera_S10Ultra_192.168.100.82` 通过 SDK 读到的 RGB 图像内参如下。该值来自 `LX_PTR_2D_INTRINSIC_PARAMETERS`，对应 SDK 当前输出的处理后 RGB 图像；如果后续改成原始未去畸变图像，需要重新核对畸变模型和畸变系数。

```yaml
cam_model: Pinhole
cam_width: 1280
cam_height: 1080
scale: 1.0
cam_fx: 410.657836914
cam_fy: 410.911346436
cam_cx: 675.708251953
cam_cy: 505.795715332
cam_d0: 0.0
cam_d1: 0.0
cam_d2: 0.0
cam_d3: 0.0
```

FAST-LIVO2 的输入话题可先按 MRDVS 当前 ROS2 驱动话题对应：

```yaml
common:
  img_topic: "/lx_camera_node/LxCamera_Rgb"
  lid_topic: "/lx_camera_node/LxCamera_Cloud"
  imu_topic: "/lx_camera_node/LxCamera_Imu"
  img_en: 1
  lidar_en: 1
```

LiDAR/ToF 到 IMU 外参，FAST-LIVO2 初始接入时可先沿用 FAST-LIO2 配置中的厂家结构设计外参：

```yaml
extrin_calib:
  extrinsic_R: [1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0]
  extrinsic_T: [0.014569, -0.002738, 0.022567]
```

该外参按 FAST-LIVO2/FAST-LIO2 的 LiDAR 到 IMU 方向填写：

```text
p_imu = extrinsic_R * p_lidar + extrinsic_T
```

LiDAR/ToF 到 RGB 相机外参需要按点云是否已经做 RGBD 对齐来选择。

推荐用于 FAST-LIVO2 的情况是启动 `lx_lidar_ros.launch.py`，使用 `is_xyz=2` 的 LiDAR 风格点云。该点云包含 `x/y/z/intensity/timestamp/row_pos/col_pos`，更适合 LIO/LIVO 算法；此时应优先使用物理 ToF 到 RGB 的外参：

```yaml
extrin_calib:
  Rcl: [0.9999749976, 0.0070269292, 0.0007914628,
        -0.0070300522, 0.9999672299, 0.0040146685,
        -0.0007632260, -0.0040201322, 0.9999916280]
  Pcl: [-0.0002534064, 0.0164201476, -0.0020634814]
```

含义为：

```text
p_camera = Rcl * p_lidar + Pcl
```

如果使用普通 `lx_camera_ros.launch.py` 且开启 `LX_INT_RGBD_ALIGN_MODE=1`，SDK 会把深度/点云对齐到 RGB 图像坐标系。该模式适合彩色点云显示，但默认 `is_xyz=1` 点云没有 FAST-LIVO2 运动补偿需要的点级 `timestamp` 字段；若仅用于验证 RGBD 对齐效果，`Rcl/Pcl` 可先按单位外参测试：

```yaml
Rcl: [1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0]
Pcl: [0.0, 0.0, 0.0]
```

时间偏移初始可先置 0，后续根据图像、点云和 IMU 的实际时间戳同步情况再调：

```yaml
time_offset:
  imu_time_offset: 0.0
  img_time_offset: 0.0
  lidar_time_offset: 0.0
  exposure_time_init: 0.0
```

注意：官方 FAST-LIVO2 是 ROS1/catkin 工程，默认示例面向 Livox Avia。MRDVS 当前是 ROS2 驱动，点云是标准 `sensor_msgs/msg/PointCloud2`，因此还需要处理 ROS2 到 ROS1 的桥接或将 FAST-LIVO2 移植到 ROS2，并适配点云字段读取 `x/y/z/intensity/timestamp`。

### 运行 MRDVS + FAST-LIVO2 ROS2

当前分支引入了 `Robotic-Developer-Road/FAST-LIVO2` 的 ROS2 移植版和配套 `rpg_vikit`，并新增 MRDVS 的标准 `PointCloud2` 预处理分支。MRDVS 点云类型编号为 `lidar_type: 8`，读取字段：

```text
x y z intensity timestamp row_pos col_pos
```

其中 `timestamp` 按 MRDVS 驱动发布的绝对微秒时间戳处理，FAST-LIVO2 内部会转换为每帧点云的相对毫秒时间，用于点云运动补偿。

构建 FAST-LIVO2 相关包：

```bash
colcon build --packages-up-to fast_livo --symlink-install
source install/setup.bash
```

一键启动 MRDVS 驱动和 FAST-LIVO2：

```bash
ros2 launch fast_livo mrdvs_full_launch.py camera_ip:=192.168.100.82 use_rviz:=True
```

该启动文件会维护 FAST-LIVO2 可视化使用的 TF 链：

```text
map -> camera_init -> aft_mapped -> mrdvs_tof -> mrdvs_rgb
```

其中 `map -> camera_init` 是静态显示变换，用于把 MRDVS/相机光学坐标显示成 ROS 常用的 `X` 前、`Y` 左、`Z` 上；`camera_init -> aft_mapped` 由 FAST-LIVO2 根据里程计结果动态发布；`aft_mapped -> mrdvs_tof` 在一体 launch 中默认使用 LiDAR_IMU_Init refinement 标定外参；`mrdvs_tof -> mrdvs_rgb` 由 MRDVS 驱动按设备 ToF/RGB 外参发布。FAST-LIVO2 的 RViz 配置默认使用 `map` 作为 Fixed Frame。

分步调试时，先启动 MRDVS LiDAR 模式：

```bash
ros2 launch lx_camera_ros lx_lidar_ros.launch.py ip:=192.168.100.82 enable_rviz:=false
```

确认 `/lx_camera_node/LxCamera_Cloud`、`/lx_camera_node/LxCamera_Rgb`、`/lx_camera_node/LxCamera_Imu` 都有数据后，再启动 FAST-LIVO2：

```bash
ros2 launch fast_livo mapping_mrdvs.launch.py use_rviz:=True
```

MRDVS 专用配置文件为：

```text
src/fast_livo/config/mrdvs.yaml
src/fast_livo/config/camera_mrdvs.yaml
```

当前配置使用前文记录的 RGB 内参、ToF/RGB 外参、LiDAR/IMU 初始外参和 MRDVS IMU Allan 标定噪声；时间偏移 `imu_time_offset`、`img_time_offset`、`lidar_time_offset` 初始均为 `0.0`，后续需要通过 rosbag 观察图像、点云和 IMU 的实际时间偏差再微调。

当前已新增 FAST-LIVO2 专用 LiDAR_IMU_Init 标定配置：

```text
src/fast_livo/config/mrdvs_lidar_imu_init.yaml
```

该配置已回退为厂家结构设计外参，并把 `time_offset.imu_time_offset` 设置为 `0.0`；`mrdvs_full_launch.py` 默认加载 `mrdvs.yaml`，同时把 `aft_mapped -> mrdvs_tof` 静态 TF 设置为同一套厂家结构外参。

为了让 MRDVS 可以不依赖 Livox 驱动独立编译，FAST-LIVO2 的 Livox `CustomMsg` 输入被改成可选项，默认关闭。MRDVS 使用标准 `PointCloud2` 路径，不需要安装 `livox_ros_driver2`。

## Codex 工作规则

当前工作空间的 Codex 用户规则写在 `AGENTS.md`。后续 Codex 在本目录内工作时，应先读取并遵循该文件。

## 更新记录

- 2026-07-09：补充 MRDVS 原始驱动两种点云启动方式：`lx_camera_ros.launch.py` 用于 RGBD 对齐彩色点云显示，`lx_lidar_ros.launch.py` 用于带强度和点级时间戳的 SLAM 点云。
- 2026-07-09：FAST-LIVO2 的 LIO 更新增加零有效约束保护；当 `effective feature num` 为 0 时不再计算 NaN 平均残差、不执行 LIO EKF 更新，也不把当前帧写入 voxel map，避免跟踪丢失后的坏帧污染地图。
- 2026-07-09：修复 FAST-LIVO2 遇到 IMU 前向大跳变后持续丢弃后续 IMU 导致卡住的问题；现在大跳变会清空旧 LiDAR/RGB/IMU 同步缓冲和 IMU 传播缓冲，并把当前 IMU 作为新的时间基准继续接收。
- 2026-07-09：FAST-LIO2 和 FAST-LIVO2 的 IMU 回调入口新增非递增时间戳过滤；相同时间戳的 IMU 样本会被丢弃，倒退时间戳仍按异常处理，避免 `dt=0` 或回跳样本进入 IMU 积分。
- 2026-07-09：确认厂家 `imu_lidar_ext` 为 LiDAR 在 IMU 坐标系下的位置后，将 FAST-LIO2 和 FAST-LIVO2 默认启动重新切回厂家结构外参；此前 LiDAR_IMU_Init refinement 平移量与设备物理结构不符，相关配置不再作为默认。
- 2026-07-08：修复 FAST-LIO2 使用 LiDAR_IMU_Init 标定配置启动后 `lio_node` abort 的问题；根因是日志截断后的 `r_il` 旋转矩阵不够正交，Sophus 构造 SO3 时会直接中止。现在加载配置时会正交化 `r_il`，并修正点到平面残差 IMU 姿态雅可比中误用 `t_wi` 的问题，新增对应 gtest。
- 2026-07-08：新增 LiDAR_IMU_Init refinement 标定配置 `mrdvs_lidar_imu_init.yaml`，同步写入 FAST-LIO2 与 FAST-LIVO2；当时用于测试 `0.067510s` IMU 时间偏移，后续因平移量与设备物理结构不符已回退为厂家结构外参。
- 2026-07-07：将 FAST-LIVO2 MRDVS 配置中的点云近距离盲区 `preprocess.blind` 从 `0.35m` 调整为 `0.15m`，用于保留更多近距离 ToF/LiDAR 点。
- 2026-07-07：将 MRDVS IMU Allan 标定的 `avg-axis` 噪声写入 FAST-LIO2 和 FAST-LIVO2 配置；FAST-LIVO2 现在会读取 `b_acc_cov`、`b_gyr_cov`，不再使用写死的 bias covariance 默认值。
- 2026-07-07：调整 FAST-LIVO2 RViz 默认 Fixed Frame 为 `map`，并在 `mapping_mrdvs.launch.py` 中维护 `map -> camera_init -> aft_mapped -> mrdvs_tof -> mrdvs_rgb` TF 链，使 MRDVS 点云按 ROS 常用 Z-up 方向显示，同时保留 FAST-LIVO2 自身 `camera_init -> aft_mapped` 动态位姿输出。
- 2026-07-07：修复 MRDVS + FAST-LIVO2 一键启动后 RViz 无数据的问题；`mapping_mrdvs.launch.py` 延迟启动 `fastlivo_mapping` 等待 `parameter_blackboard`，`vikit` 跨节点参数读取增加等待和兜底，并修复图像回调保存 `cv_bridge::toCvShare` 外部消息内存导致 VIO 段错误的问题。
- 2026-07-07：新建 `feature/fast-livo2-mrdvs-adapter` 分支，引入 FAST-LIVO2 ROS2 移植版和 `rpg_vikit`，修正 Jazzy 下 vikit/Sophus 的构建方式，新增 MRDVS `PointCloud2` 预处理、时间戳换算测试、`mrdvs.yaml`、`camera_mrdvs.yaml`、`mapping_mrdvs.launch.py` 和 `mrdvs_full_launch.py`。
- 2026-07-07：将 FAST-LIO2 默认配置从 `mrdvs_refined.yaml` 恢复为稳定基线 `mrdvs.yaml`，避免精细配置在移动时点云量过大导致卡顿；`mrdvs_refined.yaml` 保留为手动调试选项。
- 2026-07-07：修复 `mrdvs_full_launch.py` 中驱动 `enable_rviz:=false` 参数影响 FAST-LIO2 RViz 的问题；现在驱动 RViz 仍关闭，FAST-LIO2 RViz 会按一键启动的 `enable_rviz` 参数正常打开。
- 2026-07-07：新增 `fastlio2/launch/mrdvs_full_launch.py`，一键先启动固定 IP `192.168.100.82` 的 MRDVS LiDAR 驱动并关闭驱动 RViz，等待默认 3 秒后启动 FAST-LIO2 和 FAST-LIO2 RViz；`mrdvs_lio_launch.py` 默认切换为 `mrdvs_refined.yaml`，`lx_lidar_ros.launch.py` 默认固定 IP，并在固定 IP 模式下跳过 SDK 设备枚举。
- 2026-07-07：新增 `read_fastlivo_calib` 工具说明，并记录当前设备用于 FAST-LIVO2 初始接入的 RGB 内参、ToF/RGB 外参、LiDAR/IMU 初始外参，以及 RGBD 对齐开启和未开启时 `Rcl/Pcl` 的使用区别。
- 2026-07-07：新增 FAST-LIO2 稍精细配置 `mrdvs_refined.yaml`，在保留默认稳定配置 `mrdvs.yaml` 的同时，降低点云抽稀和体素分辨率以保留更多地图细节；`mrdvs_lio_launch.py` 新增 `config_file` 参数，可通过 `config_file:=mrdvs_refined.yaml` 快速切换配置。
- 2026-07-07：统一 MRDVS + FAST-LIO2 的 TF 连接为 `map -> mrdvs_imu -> mrdvs_tof`；FAST-LIO2 launch 新增 `mrdvs_imu -> mrdvs_tof` 静态 TF，LiDAR 驱动模式关闭原有 `base_link -> mrdvs_tof` TF，避免同一 child frame 有两个父节点，同时保留 `mrdvs_tof -> mrdvs_rgb` 内部 TF；将 FAST-LIO2 RViz Fixed Frame 改为 `map`，并修复 `/fastlio2/lio_path` 顶层 `header.stamp` 一直为 0 导致 RViz Message Filter 丢弃路径的问题。
- 2026-07-06：新增 `read_imu_extrinsic` 工具，通过 SDK 读取 `LX_PTR_IMU_EXTRIC_PARAM`，打印 IMU 外参原始 12 个 float、旋转矩阵、平移向量和 YAML 候选片段，并检测全 0 无效外参，用于后续与标定结果对比。
- 2026-07-07：将厂家提供的 MRDVS 结构设计外参 `imu_lidar_ext = [0.014569, -0.002738, 0.022567, 0, 0, 0]` 写入 `fastlio2/config/mrdvs.yaml`，作为 FAST-LIO2 的 LiDAR 到 IMU 初始外参。
- 2026-07-06：接入 `liangheming/FASTLIO2_ROS2` 的 `fastlio2` 主里程计包，移除 Livox 消息依赖，适配 MRDVS 的 `PointCloud2` 点云和 IMU 话题，新增 `mrdvs.yaml` 与 `mrdvs_lio_launch.py`。
- 2026-07-06：新增 `record_bag.sh`，支持传入 bag 名称并将所有 ROS2 话题录制到 `~/bag/<bag_name>`。
- 2026-07-06：开启 `lx_camera_ros.launch.py` 的 `LX_BOOL_ENABLE_IMU`，并补充 IMU 加速度与角速度量程默认参数，使 `/lx_camera_node/LxCamera_Imu` 能在重启 launch 后输出数据。
- 2026-07-06：开启 `LX_INT_RGBD_ALIGN_MODE`，使点云使用 RGB 对齐后的颜色；将相机姿态调整为 `roll=-90.0`、`pitch=0.0`、`yaw=-90.0`，把相机光学坐标旋转到 ROS `base_link` 坐标系。
- 2026-07-06：将 `lx_camera_ros.launch.py` 中的 `LX_INT_XYZ_UNIT` 从 `0` 调整为 `1`，使点云按米单位发布，避免 RViz 将毫米坐标当作米坐标显示到远处。
- 2026-07-06：新增 `AGENTS.md`，记录当前工作空间的 Codex 用户规则；新增本 `README.md`，补充项目用途、核心逻辑、结构、使用方法和更新记录；新增 `.gitignore`，避免提交 ROS2 构建产物。
