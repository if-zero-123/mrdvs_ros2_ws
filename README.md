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
- `src/fast_livo/scripts/mrdvs_imu_diagnostics.py`：实时采集 MRDVS IMU，统计频率、重复/回退时间戳、静止阈值和初始化进度，并输出 JSON、CSV 与 PNG 图表。
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

重点综合检查时间戳模式是否为 `absolute_us`、`min point - cloud header`、`point timestamp span`、负相对时间点，以及 `image - cloud_header` 和 `image - point_start`。不要只凭 `first point - cloud header` 相差几十毫秒就判定去畸变时间基准错误：`PointCloud2` 的存储首点未必是时间最早的点。当前 bag 的 `first point - cloud header` 中位数为 `+42.130ms`，但 `min point - cloud header` 为 `0ms`、没有负相对时间点且有效跨度为 `76.410ms` 到 `91.730ms`，与 strict absolute-us 契约一致。只有最早点与 header、absolute-us 数值关系或跨度本身异常时，才继续排查点时间基准；图像偏移也应结合稳定的多帧统计和现场快速运动效果评估，不要只看单帧数值直接修改 time offset。

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
| `acc_n` | `2.0331770033767380e-02` | `m/s^2` | `na` | `acc_cov` |
| `gyr_n` | `3.0946620647727048e-03` | `rad/s` | `ng` | `gyr_cov` |
| `acc_w` | `5.4152704615929208e-04` | `m/s^2` | `nba` | `b_acc_cov` |
| `gyr_w` | `4.2000451972629459e-05` | `rad/s` | `nbg` | `b_gyr_cov` |

这些值已同步写入 FAST-LIO2 的 `src/fastlio2/config/mrdvs.yaml`、`src/fastlio2/config/mrdvs_refined.yaml`、`src/fastlio2/config/mrdvs_lidar_imu_init.yaml`，以及 FAST-LIVO2 的 `src/fast_livo/config/mrdvs.yaml`、`src/fast_livo/config/mrdvs_lidar_imu_init.yaml`。FAST-LIVO2 源码也已改为读取 `b_acc_cov`、`b_gyr_cov` 配置项，不再使用写死的默认值。

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

分别构建一体启动所需的 MRDVS 驱动和 FAST-LIVO2 依赖闭包；驱动先清理已有 CMake 安装模式缓存，FAST-LIVO2 保持 symlink-install：

```bash
colcon build --packages-select lx_camera_ros --cmake-clean-cache --cmake-args -DBUILD_TESTING=ON
colcon build --packages-up-to fast_livo --symlink-install --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

一键启动 MRDVS 驱动和 FAST-LIVO2：

```bash
source install/setup.bash
ros2 launch fast_livo mrdvs_full_launch.py \
  camera_ip:=192.168.100.82 \
  imu_angular_range_level:=2 \
  use_rviz:=True
```

`imu_angular_range_level` 的合法范围是 `0..4`，默认 `2`，对应约 `±500 deg/s`。只有现场日志或原始数据确认快速手持动作超过该范围时才提高一级；最终量程和运行模式稳定后，需要重新录制 1 到 2 小时静止 IMU 数据并重做 Allan 标定。

启动后必须让设备连续静止约 3 秒，等待控制台明确输出 IMU 初始化完成后再开始较快旋转或平移；窗口内发生运动、出现非有限样本或超过 `0.2s` 的时间大跳变会清空已有样本并重新累计 600 个连续静止样本。SDK 偶发的重复时间戳和不超过 `0.2s` 的小幅回退只会丢弃当前样本，不会再清空已经累计的静止进度。初始化期间同步时间水位会继续前移，完成后不会回放初始化前的旧点云。

#### IMU 时间戳与初始化诊断

`mrdvs_imu_diagnostics` 可直接订阅真实 IMU 话题，按 FAST-LIVO2 当前阈值统计时间戳和静止质量，并同时回放修复前、修复后的初始化进度规则。先在终端一关闭 RViz 启动驱动和 FAST-LIVO2：

```bash
source install/setup.bash
ros2 launch fast_livo mrdvs_full_launch.py \
  camera_ip:=192.168.100.82 \
  imu_angular_range_level:=2 \
  use_rviz:=False
```

设备保持静止，在终端二采样 30 秒：

```bash
source install/setup.bash
OUTPUT_DIR=/tmp/mrdvs_imu_diagnostics_$(date +%Y%m%d-%H%M%S)
ros2 run fast_livo mrdvs_imu_diagnostics \
  --duration 30 \
  --output-dir "$OUTPUT_DIR"
```

输出目录包含 `summary.json`、逐样本 `samples.csv` 和四联图 `imu_diagnostics.png`。具体实测结果应单独归档，不写入项目 README。

LiDAR launch 会以 SLAM 模式显式关闭 RGBD 对齐和 3D 反畸变，并在每次启流前读回陀螺量程及这两个开关；设置失败、读回失败或值不一致都会拒绝 `DcStartStream`。FAST-LIVO2 仍订阅 `/lx_camera_node/LxCamera_Rgb`，使用物理 ToF-to-RGB 外参为点云着色，不依赖 SDK 的 RGBD 对齐点云。

MRDVS 预处理只接受驱动契约中的 absolute-us 点时间：先过滤非有限坐标、零点和小于 `0.19m` 的近场点，float 坐标恰好 `0.19m` 时保留；恰好 `200ms` 的帧内偏移合法，负数、非有限值、错误时间单位/epoch 或超过 `200ms` 的点会丢弃。每 100 帧限频汇总一次各类丢弃计数；过滤后少于 2 点的整帧不会进入同步缓冲、IMU 去畸变或 voxel map 更新。

该启动文件会维护 FAST-LIVO2 可视化使用的 TF 链：

```text
map -> camera_init -> aft_mapped -> mrdvs_tof -> mrdvs_rgb
```

其中 `map -> camera_init` 是静态显示变换，用于把 MRDVS/相机光学坐标显示成 ROS 常用的 `X` 前、`Y` 左、`Z` 上；`camera_init -> aft_mapped` 由 FAST-LIVO2 根据里程计结果动态发布；`aft_mapped -> mrdvs_tof` 在一体 launch 中默认使用厂家结构外参，不再默认使用 LiDAR_IMU_Init refinement 外参；`mrdvs_tof -> mrdvs_rgb` 由 MRDVS 驱动按设备 ToF/RGB 外参发布。FAST-LIVO2 的 RViz 配置默认使用 `map` 作为 Fixed Frame。

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

### FAST-LIVO2 手持扫描稳定性修复设计（2026-07-10）

本轮修复面向“启动后可静止初始化、随后进行较快手持旋转和平移扫描”的使用方式。现有 RViz 的历史点云累计用于人工观察，保持不变；修复范围只包含传感器配置、MRDVS 点云预处理和 IMU 初始化链路。

驱动配置设计：

- MRDVS LiDAR 模式默认把 IMU 角速度量程设为 `level 2`，即约 `±500 deg/s`，并通过 launch 参数保留现场调整能力。
- LiDAR/SLAM 模式显式关闭 `LX_INT_RGBD_ALIGN_MODE` 和 `LX_BOOL_ENABLE_3D_UNDISTORT`，启动后读回并记录角速度量程及这两个开关的实际值，避免依赖设备遗留状态。SDK 已说明这两项功能开启后 `XYZIRT` 的逐点时间戳不准确；关键参数设置失败或读回值不一致时，节点应明确报错并停止进入 SLAM 数据流。
- RGB、IMU 和带 `timestamp/row_pos/col_pos` 的 `XYZIRT` 点云仍按现有话题发布；RGB 着色继续使用 FAST-LIVO2 中的物理 ToF-to-RGB 外参，不依赖 RGBD 对齐点云。

点云预处理设计：

- 修复 `preprocess.blind` 与 `blind_sqr` 不同步的问题，距离判断直接使用由当前 `blind` 计算出的平方阈值。
- 依次过滤非有限坐标、零点、近场点和无效逐点时间；负时间、未知时间单位、无法换算的时间或超过可配置 `preprocess.max_point_time_offset_ms` 的帧内时间不再静默压成 `0ms`。MRDVS 默认最大帧内时间设为 `200ms`，覆盖当前实测不超过约 `92ms` 的扫描跨度。
- 对丢弃的近场点和时间异常点做限频统计输出；若一帧过滤后少于 2 个有效点，则整帧跳过，不进入 IMU 去畸变和 voxel map 更新。
- 保留有效点的真实时间排序和逐点去畸变流程；不使用简单的 RViz 历史累计或 `scan_line` 参数掩盖输入数据问题。

IMU 初始化设计：

- 启动 FAST-LIVO2 后，设备连续静止约 3 秒，累计约 600 个被接受的严格递增时间戳 IMU 样本后再结束初始化；重复时间戳和不超过 `0.2s` 的小幅回退样本会被丢弃，但保留已有初始化进度，前后方向超过 `0.2s` 的时间跳变才重置传感器同步流和静止窗口。
- 初始化窗口要求连续样本满足 `|gyro| <= 0.10rad/s` 且 `||acc||` 与 `9.81m/s^2` 的偏差不超过 `0.75m/s^2`；两个阈值均作为 MRDVS IMU 配置项保留调整能力。任一条件不满足时重新累计完整静止窗口，避免把手持动作写入重力和 bias 初值。
- 使用静止窗口的平均角速度初始化 gyro bias，不再计算 `mean_gyr` 后仍强制写入零 bias。
- 当前 Allan 参数先保留；最终量程和运行模式稳定后，重新录制 1 到 2 小时静止 IMU 数据并更新噪声参数。

验证标准：

- 使用测试驱动方式覆盖近场阈值、零点、NaN/Inf、合法与非法逐点时间、IMU 重复/回退/大跳变决策、静止窗口判断和 gyro bias 初始化。
- 构建并测试 `lx_camera_ros`、`fast_livo`，检查 launch 参数传递和安装目录内容。
- 硬件验收时先静止初始化，再执行慢速与快速手持旋转/平移；确认 IMU 不再贴量程边界，当前配准点云没有随曝光分区产生明显拉线，重新静止后位姿不继续发散。
- 本设计不修改 `src/fast_livo/rviz_cfg/fast_livo2.rviz`，也不改变用户现有的历史点云观察方式。

### FAST-LIVO2 手持扫描稳定性实施计划

> **执行要求：** 使用 `subagent-driven-development` 或 `executing-plans` 按阶段执行；每个生产代码修改必须先有能正确失败的回归测试。

**目标：** 在保留现有 RViz 观察方式的前提下，消除 MRDVS 快速手持扫描中的陀螺饱和、无效近场点、异常逐点时间和不可靠 IMU 初值。

**架构：** 驱动层负责设置并校验 SDK 关键模式；FAST-LIVO2 预处理层只接收空间坐标和逐点时间均有效的点；独立的 IMU 初始化累加器负责连续静止窗口、均值和 gyro bias。三个阶段分别测试、构建和提交，最后再做整体离线与硬件验收。

**技术栈：** ROS2 Jazzy、C++17、Eigen、PCL、ament/GoogleTest、Python ROS2 launch。

#### 全局约束

- 不修改 `src/fast_livo/rviz_cfg/fast_livo2.rviz`。
- MRDVS LiDAR 模式默认角速度量程为 `level 2`，约 `±500 deg/s`。
- RGBD 对齐和 3D 反畸变在 LiDAR/SLAM 模式下必须显式为关闭状态。
- 近场阈值为 `0.19m`，最大帧内点时间默认 `200ms`。
- IMU 初始化需要 600 个连续静止样本；静止阈值为 `|gyro| <= 0.10rad/s`、`abs(||acc|| - 9.81) <= 0.75m/s^2`。
- 项目说明、计划和更新记录只写入本 `README.md`，不创建其他 Markdown 文件。

#### 阶段一：设置并校验 MRDVS SLAM 传感器模式

**文件：**

- 新建 `src/lx_camera_ros/src/lx_camera/slam_sensor_settings.h`
- 新建 `src/lx_camera_ros/test/test_slam_sensor_settings.cpp`
- 修改 `src/lx_camera_ros/src/lx_camera/lx_camera.h`
- 修改 `src/lx_camera_ros/src/lx_camera/lx_camera.cpp`
- 修改 `src/lx_camera_ros/launch/lx_lidar_ros.launch.py`
- 修改 `src/fast_livo/launch/mrdvs_full_launch.py`
- 修改 `src/lx_camera_ros/CMakeLists.txt`

**接口：**

```cpp
namespace lx_camera_ros {
bool isValidImuAngularRangeLevel(int level);
bool criticalSensorSettingMatches(int requested, int actual);
}

bool LxCamera::VerifyCriticalIntParameter(int command, const char *name, int expected);
bool LxCamera::VerifyCriticalBoolParameter(int command, const char *name, bool expected);
```

- [x] 先在 `test_slam_sensor_settings.cpp` 注册并写入以下失败测试：

```cpp
#include "lx_camera/slam_sensor_settings.h"
#include <gtest/gtest.h>

TEST(SlamSensorSettings, AcceptsOnlySdkAngularRangeLevels)
{
  for (int level = 0; level <= 4; ++level)
    EXPECT_TRUE(lx_camera_ros::isValidImuAngularRangeLevel(level));
  EXPECT_FALSE(lx_camera_ros::isValidImuAngularRangeLevel(-1));
  EXPECT_FALSE(lx_camera_ros::isValidImuAngularRangeLevel(5));
}

TEST(SlamSensorSettings, RequiresReadbackToMatchRequestedValue)
{
  EXPECT_TRUE(lx_camera_ros::criticalSensorSettingMatches(2, 2));
  EXPECT_FALSE(lx_camera_ros::criticalSensorSettingMatches(2, 1));
}
```

- [x] 运行 `colcon build --packages-select lx_camera_ros --cmake-args -DBUILD_TESTING=ON`，确认因 `slam_sensor_settings.h` 或目标函数不存在而编译失败。
- [x] 实现纯函数和驱动读回校验；`DcGetIntValue`/`DcGetBoolValue` 失败或实际值不一致时通过 `RCLCPP_ERROR` 记录明确错误，并返回 `false`/`LX_ERROR`，阻止节点调用 `DcStartStream`，不依赖 C++ 异常。
- [x] 在 `lx_lidar_ros.launch.py` 声明 `imu_angular_range_level`，默认值为 `2`，并传入以下参数：

```python
{"LX_INT_IMU_ANGULAR_RANGE_LEVEL": ParameterValue(imu_angular_range_level, value_type=int)},
{"LX_INT_RGBD_ALIGN_MODE": 0},
{"LX_BOOL_ENABLE_3D_UNDISTORT": 0},
```

- [x] 在 `mrdvs_full_launch.py` 声明同名参数并只向 `lx_lidar_ros.launch.py` 透传，保持 FAST-LIVO2 YAML 和 RViz 参数不变。
- [x] 运行目标 gtest、`python3 -m py_compile` 检查两个 launch 文件，并用 `ros2 launch fast_livo mrdvs_full_launch.py --show-args` 确认量程参数默认值和覆盖入口存在。
- [x] 提交阶段一，提交信息使用 `fix: enforce MRDVS SLAM sensor settings`。

#### 阶段二：修复 MRDVS 近场点与逐点时间过滤

**文件：**

- 新建 `src/fast_livo/include/mrdvs_preprocess_utils.h`
- 修改 `src/fast_livo/include/mrdvs_time_utils.h`
- 修改 `src/fast_livo/include/preprocess.h`
- 修改 `src/fast_livo/src/preprocess.cpp`
- 修改 `src/fast_livo/src/LIVMapper.cpp`
- 修改 `src/fast_livo/config/mrdvs.yaml`
- 修改 `src/fast_livo/config/mrdvs_lidar_imu_init.yaml`
- 修改 `src/fast_livo/test/test_mrdvs_time_utils.cpp`
- 新建 `src/fast_livo/test/test_mrdvs_preprocess.cpp`
- 修改 `src/fast_livo/CMakeLists.txt`，注册两个阶段二测试目标

**接口：**

```cpp
namespace fast_livo {
enum class MrdvsPointStatus { kValid, kNonFinite, kZeroOrNear };
MrdvsPointStatus classifyMrdvsPoint(double x, double y, double z, double blind_m);

enum class MrdvsTimestampStatus {
  kValid, kNonFinite, kUnknownUnit, kNegative, kTooLarge
};
struct MrdvsTimestampResult {
  MrdvsTimestampStatus status;
  double relative_ms;
  bool valid() const { return status == MrdvsTimestampStatus::kValid; }
};
MrdvsTimestampResult parseMrdvsTimestamp(
  double raw_timestamp, double cloud_start_sec, double max_offset_ms);
}

void Preprocess::setBlind(double blind_m);
```

- [x] 先在 `test_mrdvs_preprocess.cpp` 写入空间边界和真实 `mrdvs_handler()` 测试，在 `test_mrdvs_time_utils.cpp` 写入 strict absolute-us 时间测试，并由 CMake 分别注册 `test_mrdvs_preprocess` 和 `test_mrdvs_time_utils`。代表性测试如下：

```cpp
// test_mrdvs_preprocess.cpp：空间分类；同文件还覆盖非有限坐标和真实 handler 的过滤与排序。
TEST(MrdvsPreprocessUtils, ClassifiesZeroNearBoundaryAndFarPoints)
{
  using fast_livo::MrdvsPointStatus;
  EXPECT_EQ(fast_livo::classifyMrdvsPoint(0.0, 0.0, 0.0, 0.19),
            MrdvsPointStatus::kZeroOrNear);
  EXPECT_EQ(fast_livo::classifyMrdvsPoint(0.18, 0.0, 0.0, 0.19),
            MrdvsPointStatus::kZeroOrNear);
  EXPECT_EQ(fast_livo::classifyMrdvsPoint(0.19, 0.0, 0.0, 0.19),
            MrdvsPointStatus::kValid);
  EXPECT_EQ(fast_livo::classifyMrdvsPoint(0.20, 0.0, 0.0, 0.19),
            MrdvsPointStatus::kValid);
}

TEST(MrdvsTimeUtils, ParsesAbsoluteMicrosecondsAtFrameStart)
{
  constexpr double cloud_start_sec = 1000000.0;
  const auto result = fast_livo::parseMrdvsTimestamp(
    cloud_start_sec * 1.0e6, cloud_start_sec, 200.0);
  EXPECT_EQ(result.status, fast_livo::MrdvsTimestampStatus::kValid);
  EXPECT_DOUBLE_EQ(result.relative_ms, 0.0);
}

TEST(MrdvsTimeUtils, AcceptsConfiguredMaximumAndRejectsOnlyValuesAboveIt)
{
  constexpr double cloud_start_sec = 1000000.0;
  constexpr double cloud_start_us = cloud_start_sec * 1.0e6;
  const auto at_limit = fast_livo::parseMrdvsTimestamp(
    cloud_start_us + 200000.0, cloud_start_sec, 200.0);
  const auto above_limit = fast_livo::parseMrdvsTimestamp(
    cloud_start_us + 250000.0, cloud_start_sec, 200.0);
  EXPECT_EQ(at_limit.status, fast_livo::MrdvsTimestampStatus::kValid);
  EXPECT_DOUBLE_EQ(at_limit.relative_ms, 200.0);
  EXPECT_EQ(above_limit.status, fast_livo::MrdvsTimestampStatus::kTooLarge);
}

TEST(MrdvsTimeUtils, RejectsRelativeOrWrongEpochValuesAsUnknownUnit)
{
  constexpr double cloud_start_sec = 1000000.0;
  EXPECT_EQ(fast_livo::parseMrdvsTimestamp(0.0, cloud_start_sec, 200.0).status,
            fast_livo::MrdvsTimestampStatus::kUnknownUnit);
  EXPECT_EQ(fast_livo::parseMrdvsTimestamp(250000.0, cloud_start_sec, 200.0).status,
            fast_livo::MrdvsTimestampStatus::kUnknownUnit);
  EXPECT_EQ(fast_livo::parseMrdvsTimestamp(1.0e9, cloud_start_sec, 200.0).status,
            fast_livo::MrdvsTimestampStatus::kUnknownUnit);
}
```

- [x] 运行 `colcon build --packages-select fast_livo --cmake-args -DBUILD_TESTING=ON`，确认新类型和函数不存在导致编译失败。
- [x] 实现两个纯工具；保留旧的 `mrdvsTimestampToRelativeMs` 兼容入口，但 MRDVS 生产路径只使用带状态的 `parseMrdvsTimestamp`。
- [x] 构造函数、`Preprocess::set()` 和 `LIVMapper` 参数加载统一调用 `setBlind()`，保证 `blind_sqr = blind * blind`；新增并读取 `preprocess.max_point_time_offset_ms: 200.0`。
- [x] 重写 `mrdvs_handler()` 的单点判断顺序：空间状态、时间状态、构造点、按相对毫秒排序；每 100 帧汇总一次近场和时间异常丢弃数量。
- [x] `standard_pcl_cbk()` 在预处理输出少于 2 点时不写入 LiDAR 缓冲，避免后续访问空点云的 `back()`。
- [x] 运行目标 gtest、全部 `fast_livo` gtest，并构建 `fast_livo`。
- [x] 提交阶段二，提交信息使用 `fix: validate MRDVS points and timestamps`。

#### 阶段三：实现连续静止 IMU 初始化和 gyro bias 初值

**文件：**

- 新建 `src/fast_livo/include/imu_initialization_utils.h`
- 新建 `src/fast_livo/test/test_imu_initialization_utils.cpp`
- 修改 `src/fast_livo/include/IMU_Processing.h`
- 修改 `src/fast_livo/src/IMU_Processing.cpp`
- 修改 `src/fast_livo/include/LIVMapper.h`
- 修改 `src/fast_livo/src/LIVMapper.cpp`
- 修改 `src/fast_livo/config/mrdvs.yaml`
- 修改 `src/fast_livo/config/mrdvs_lidar_imu_init.yaml`
- 修改 `src/fast_livo/CMakeLists.txt`

**接口：**

```cpp
namespace fast_livo {
struct ImuInitializationConfig {
  int required_samples;
  double gravity_magnitude;
  double max_gyro_norm;
  double max_acc_norm_error;
};

class ImuInitializationAccumulator {
public:
  explicit ImuInitializationAccumulator(const ImuInitializationConfig &config);
  // 返回 true 表示当前样本满足静止条件并被累计；返回 false 表示窗口已重置。
  bool addSample(const Eigen::Vector3d &acc, const Eigen::Vector3d &gyro);
  void reset();
  bool ready() const;
  int sampleCount() const;
  const Eigen::Vector3d &meanAcc() const;
  const Eigen::Vector3d &meanGyro() const;
};
}

void ImuProcess::configure_imu_initialization(
  bool stationary_init_en,
  const fast_livo::ImuInitializationConfig &config);
void ImuProcess::reset_imu_initialization_window();
```

- [x] 先注册 `test_imu_initialization_utils` 并写入以下失败测试：

```cpp
TEST(ImuInitializationAccumulator, RequiresConsecutiveStationarySamples)
{
  fast_livo::ImuInitializationAccumulator acc({3, 9.81, 0.10, 0.75});
  const Eigen::Vector3d static_acc(0.0, 0.0, 9.70);
  EXPECT_TRUE(acc.addSample(static_acc, Eigen::Vector3d(0.01, -0.02, 0.005)));
  EXPECT_TRUE(acc.addSample(static_acc, Eigen::Vector3d(0.02, -0.01, 0.005)));
  EXPECT_FALSE(acc.ready());
  EXPECT_TRUE(acc.addSample(static_acc, Eigen::Vector3d(0.00, -0.03, 0.005)));
  EXPECT_TRUE(acc.ready());
  EXPECT_EQ(acc.sampleCount(), 3);
  EXPECT_TRUE(acc.meanGyro().isApprox(Eigen::Vector3d(0.01, -0.02, 0.005), 1e-12));
}

TEST(ImuInitializationAccumulator, MotionResetsTheWindow)
{
  fast_livo::ImuInitializationAccumulator acc({3, 9.81, 0.10, 0.75});
  EXPECT_TRUE(acc.addSample(Eigen::Vector3d(0.0, 0.0, 9.70),
                            Eigen::Vector3d(0.01, 0.0, 0.0)));
  EXPECT_FALSE(acc.addSample(Eigen::Vector3d(0.0, 0.0, 9.70),
                             Eigen::Vector3d(0.20, 0.0, 0.0)));
  EXPECT_EQ(acc.sampleCount(), 0);
}
```

- [x] 运行 `colcon build --packages-select fast_livo --cmake-args -DBUILD_TESTING=ON`，确认因累加器尚不存在而编译失败。
- [x] 实现独立累加器并接入 `ImuProcess::Reset()`、`configure_imu_initialization()`、`reset_imu_initialization_window()` 和 `IMU_init()`；只在累加器拥有有效样本时更新重力，完成时写入 `state_inout.bias_g = mean_gyr`。
- [x] 新增并读取 `imu.imu_init_max_gyr_norm` 和 `imu.imu_init_acc_norm_tolerance`，MRDVS 两份配置分别写入 `600`、`0.10`、`0.75`；样本数达到 `required_samples` 时 `ready()` 返回 true，之后的连续静止样本不再改变均值或计数，MRDVS 窗口冻结在 600 个样本。
- [x] 运行新 gtest、全部 `fast_livo` 测试并构建 `fast_livo`。
- [x] 提交阶段三，提交信息使用 `fix: require stationary IMU initialization`。

#### 阶段四：集成验证、使用说明和最终提交

**文件：**

- 修改 `README.md`
- 不修改 `src/fast_livo/rviz_cfg/fast_livo2.rviz`

- [x] 运行完整构建：

```bash
colcon build --packages-up-to fast_livo --symlink-install --cmake-args -DBUILD_TESTING=ON
```

- [x] 运行完整测试并确认无失败：

```bash
colcon test --packages-select lx_camera_ros fast_livo --event-handlers console_direct+
colcon test-result --verbose
```

- [x] 检查 launch 语法、参数和安装目录：

```bash
python3 -m py_compile src/lx_camera_ros/launch/lx_lidar_ros.launch.py
python3 -m py_compile src/fast_livo/launch/mrdvs_full_launch.py
source install/setup.bash
ros2 launch fast_livo mrdvs_full_launch.py --show-args
ros2 launch lx_camera_ros lx_lidar_ros.launch.py --show-args
```

- [x] 重新运行现有 bag 时间分析，确认输入仍为约 10Hz 点云、约 200Hz IMU，且当前有效逐点时间跨度落在 `200ms` 上限内：

```bash
source install/setup.bash
tools/analyze_mrdvs_bag.py /home/zero/bag/mrdvs_livo_debug_20260708_155719 --max-clouds 300 --point-stride 4
```

- [x] 使用 `git diff --exit-code 2b00c0d -- src/fast_livo/rviz_cfg/fast_livo2.rviz` 确认 RViz 配置相对设计基线没有变化。
- [x] 更新本节状态、启动方式、量程覆盖示例、3 秒静止要求、验证结果和未完成的硬件验收风险。
- [x] 使用 `docs: document handheld FAST-LIVO stability fixes` 创建本地文档提交；按本轮约束不 push、不 tag。
- [ ] 使用真实 MRDVS 完成静止初始化后的慢速/快速手持旋转和平移验收，并在 RK3588 上完成 ARM 原生构建和运行确认。

#### 阶段四软件验证证据（2026-07-10）

本节只记录软件与离线数据验证完成，不代表真实 MRDVS 快速手持漂移已经在硬件上解决。

- 完整构建命令退出码为 0，`vikit_common`、`vikit_ros`、`fast_livo` 共 3 个包完成；本机为 `x86_64`。
- `colcon test` 中 `lx_camera_ros` 的 31 个 CTest 目标和 `fast_livo` 的 3 个 CTest 目标全部通过；`colcon test-result --verbose` 汇总为 `828 tests, 0 errors, 0 failures, 255 skipped`。
- 两个 launch 均通过 `py_compile`；两个 `--show-args` 都列出 `imu_angular_range_level`，合法范围 `0..4`、默认 `2`，外层 launch 只向 MRDVS 驱动透传该值。
- 两份 MRDVS 源 YAML 与 symlink-install 安装产物内容一致：`max_point_time_offset_ms=200.0`、`imu_int_frame=600`、`stationary_init_en=true`、`imu_init_max_gyr_norm=0.10`、`imu_init_acc_norm_tolerance=0.75`。四个 Allan 数值仍为 `2.0348936872780068e-02`、`2.3162072810468197e-03`、`4.5452036338894608e-04`、`2.3553527673141791e-05`；其他数据集没有启用静止初始化，外参和三个 time offset 均未调整。
- `pre`、`imu_proc`、`test_mrdvs_time_utils`、`test_mrdvs_preprocess`、`test_imu_initialization_utils` 的实际构建 flags 都含 `-fno-fast-math`。这些是 x86_64 证据，RK3588 ARM 仍需原生确认。
- 指定 bag 含 868 帧点云、17350 帧 IMU、868 帧 RGB；按 header 平均间隔换算约为点云 `9.996Hz`、IMU `199.680Hz`、RGB `9.994Hz`。300 帧抽样的点时间跨度最小/平均/中位/p95/最大分别为 `76.410/78.363/76.410/84.070/91.730ms`，有效跨度全部小于 `200ms`，没有负相对时间点。
- bag 中 `min point - cloud header` 中位数为 `0ms`，`first point - cloud header` 中位数为 `42.130ms`；最近 RGB/IMU 相对 cloud header 的中位数分别为 `-32.341ms` 和 `+0.010ms`，相对 point start 的中位数分别为 `-33.251ms` 和 `+0.008ms`。本轮只记录这些关系，没有调整任何 time offset。
- `fast_livo2.rviz` 相对设计基线 `2b00c0d` 的 diff 为空；RViz 历史累计保持不变，本轮没有修改该文件。
- 未完成风险：尚未使用真实 MRDVS 验证快速手持时是否仍会陀螺饱和、点云拉线或重新静止后继续发散，也尚未在 RK3588 上验证 ARM 原生编译与运行；因此当前不能声称硬件漂移问题已经解决。

### FAST-LIO2 MRDVS 在线回环设计与实施计划（2026-07-15）

> **执行要求：** 使用 `executing-plans` 在当前会话逐阶段实施；新增行为必须先写测试并确认测试因功能缺失而失败，再写最小实现。上游 PGO 核心算法保持可追溯，不在首轮接入中重写。

**目标：** 在不影响现有 FAST-LIO2 无回环模式和 FAST-LIVO2 的前提下，为 MRDVS + FAST-LIO2 增加运行中实时回环修正、回环可视化和优化地图保存能力。

**架构：** FAST-LIO2 使用局部坐标 `lio_local` 连续输出 `lio_local -> mrdvs_imu`；PGO 同步 `/fastlio2/body_cloud` 与 `/fastlio2/lio_odom`，沿用上游关键帧、位置候选、ICP 和 GTSAM/iSAM2 流程，发布 `map -> lio_local` 全局修正。最终 TF 为 `map -> lio_local -> mrdvs_imu -> mrdvs_tof`，回环不会修改 FAST-LIO2 的滤波状态。

**技术栈：** ROS2 Jazzy、C++17、PCL、Eigen、GTSAM 4.2、yaml-cpp、ament/GoogleTest、Python launch/pytest。上游基线为 `liangheming/FASTLIO2_ROS2@f516daac08bc46e50e814a2e7d6c8352ed8141bb`。

#### 全局约束

- 首轮接入只引入上游 `interface` 和 `pgo`，不引入 `localizer`、`hba`，不重写 `SimplePGO` 的关键帧、ICP 或 iSAM2 算法。
- 不修改 `src/fast_livo/` 下任何文件，也不在 PGO launch 中启动 FAST-LIVO2。
- 不改变 `src/fastlio2/config/mrdvs.yaml`、`mrdvs_refined.yaml` 的无回环行为；回环使用独立的 `mrdvs_pgo.yaml`。
- PGO 输入固定为 `/fastlio2/body_cloud` 和 `/fastlio2/lio_odom`，全局坐标为 `map`，FAST-LIO2 局部坐标为 `lio_local`。
- 现有 `ros2 launch fastlio2 mrdvs_full_launch.py` 保持可用；回环使用独立入口 `ros2 launch pgo mrdvs_pgo_full_launch.py`。
- 项目说明、计划和更新记录只写入本 `README.md`，不新增其他 Markdown 文档。
- 每个可构建、可回退阶段单独提交；不提交 `build/`、`install/`、`log/`、bag 或运行时地图。

#### 阶段一：导入可追溯的上游 PGO 基线

**文件：**

- 新建 `src/interface/`：保留上游 5 个服务定义及 ROS2 接口包构建文件。
- 新建 `src/pgo/`：保留上游 PGO 源码、默认配置、launch、RViz 和 MIT License。
- 新建 `src/pgo/test/test_mrdvs_pgo_contract.py`：验证上游包布局和来源基线。

**步骤：**

- [x] 先创建契约测试，断言 `src/interface/srv/SaveMaps.srv`、`src/pgo/src/pgo_node.cpp`、`src/pgo/src/pgos/simple_pgo.cpp` 和两个 `package.xml` 存在；运行 `python3 -m pytest -q src/pgo/test/test_mrdvs_pgo_contract.py`，确认因文件尚未导入而失败。
- [x] 从上游提交 `f516daac08bc46e50e814a2e7d6c8352ed8141bb` 导入 `interface/` 和 `pgo/`，不导入上游 FAST-LIO2、localizer 或 HBA，也不覆盖当前 MRDVS FAST-LIO2。
- [x] 重跑契约测试，确认上游包布局通过，并用 `git diff --no-index` 核对首轮导入的 PGO 核心文件与上游一致。
- [x] 安装 `libgtsam-dev`，运行 `colcon build --packages-select interface pgo --symlink-install --cmake-args -DBUILD_TESTING=ON`，只修复 Jazzy/Noble 的构建兼容问题，不改变算法行为。
- [x] 提交 `feat: import upstream FAST-LIO2 PGO`。

阶段一在 Ubuntu 24.04 上安装了 `libgtsam-dev 4.2.0+dfsg-1build1`。该包的 `GTSAMConfig.cmake` 引用了未随 Debian 包发布的 `libCppUnitLite.a`，因此 PGO CMake 改为直接查找已安装的 GTSAM 头文件和 `libgtsam.so` 并显式链接 TBB；独立编译链接探针和最终 `pgo_node` 动态库检查均通过。`pgo_node.cpp`、`simple_pgo.cpp`、`simple_pgo.h` 与上游固定提交保持一致。

#### 阶段二：用测试驱动 MRDVS 配置和 TF 契约

**文件：**

- 新建 `src/fastlio2/config/mrdvs_pgo.yaml`：复制当前稳定 `mrdvs.yaml` 的传感器、噪声和外参参数，只将 `world_frame` 改为 `lio_local`。
- 新建 `src/pgo/config/mrdvs.yaml`：写入 MRDVS 话题、`map`/`lio_local` 坐标和上游首轮阈值。
- 修改 `src/pgo/test/test_mrdvs_pgo_contract.py`：解析并比较两份 YAML。
- 修改 `src/pgo/CMakeLists.txt`、`src/pgo/package.xml`：将 Python 契约测试注册到 `colcon test`。

**配置接口：**

```yaml
# src/pgo/config/mrdvs.yaml
cloud_topic: /fastlio2/body_cloud
odom_topic: /fastlio2/lio_odom
map_frame: map
local_frame: lio_local
key_pose_delta_deg: 10
key_pose_delta_trans: 0.5
loop_search_radius: 1.0
loop_time_tresh: 60.0
loop_score_tresh: 0.15
loop_submap_half_range: 5
submap_resolution: 0.1
min_loop_detect_duration: 5.0
```

**步骤：**

- [ ] 先扩展测试，断言 `mrdvs_pgo.yaml` 除 `world_frame=lio_local` 外与稳定配置一致，PGO MRDVS 配置与上方接口逐项一致；确认测试因文件缺失而失败。
- [ ] 写入两份最小配置，注册 `ament_cmake_pytest`，重跑目标 pytest 和 `colcon test --packages-select pgo`。
- [ ] 使用脚本确认 `src/fast_livo/` 和原有 FAST-LIO2 MRDVS 配置相对阶段开始的 Git 基线无改动。
- [ ] 提交 `feat: add MRDVS PGO frame configuration`。

#### 阶段三：新增独立一体启动入口

**文件：**

- 新建 `src/pgo/launch/mrdvs_pgo_full_launch.py`：启动 MRDVS LiDAR 模式、FAST-LIO2 `mrdvs_pgo.yaml`、PGO `mrdvs.yaml` 和可选 PGO RViz。
- 修改 `src/pgo/test/test_mrdvs_pgo_contract.py`：验证 launch 默认值和组件边界。

**启动接口：**

```bash
ros2 launch pgo mrdvs_pgo_full_launch.py \
  camera_ip:=192.168.100.82 \
  fastlio_delay:=3.0 \
  enable_rviz:=true
```

**步骤：**

- [ ] 先扩展测试，断言 launch 默认引用 `mrdvs_pgo.yaml`、PGO 的 `mrdvs.yaml`、MRDVS 驱动和 PGO RViz，且不包含 `fast_livo`；确认测试因 launch 缺失而失败。
- [ ] 实现独立 launch，复用现有 `lx_lidar_ros.launch.py` 和 `mrdvs_lio_launch.py`，保持 `mrdvs_imu -> mrdvs_tof` 厂家结构外参不变。
- [ ] 运行 pytest、`python3 -m py_compile src/pgo/launch/mrdvs_pgo_full_launch.py` 和 `ros2 launch pgo mrdvs_pgo_full_launch.py --show-args`。
- [ ] 提交 `feat: launch MRDVS FAST-LIO2 with online PGO`。

#### 阶段四：集成验证、使用说明和硬件验收

- [ ] 构建 `lx_camera_ros`、`interface`、`fastlio2`、`pgo`，运行这些包的全部测试并用 `colcon test-result --verbose` 确认无失败。
- [ ] 启动后检查 `/fastlio2/body_cloud`、`/fastlio2/lio_odom` 的频率和时间戳同步，检查 TF 只有 `map -> lio_local -> mrdvs_imu -> mrdvs_tof` 一条父子链。
- [ ] 真实设备静止初始化后沿闭合路线运行超过 60 秒并回到起点，确认 `/pgo/loop_markers` 出现回环边、`map -> lio_local` 发生有限修正且节点不退出。
- [ ] 调用 `ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: '/tmp/mrdvs_pgo_map', save_patches: true}"`，确认生成 `map.pcd`、`poses.txt` 和 `patches/`；这些运行产物不提交。
- [ ] 只有在上游默认阈值实测漏检或误检时才调整 `loop_search_radius`、`loop_score_tresh` 或 `submap_resolution`，并记录调整证据。
- [ ] 在本 README 补充启动、保存地图、回环观察、已验证范围和未完成硬件风险，提交 `docs: document MRDVS online PGO workflow`。

## Codex 工作规则

当前工作空间的 Codex 用户规则写在 `AGENTS.md`。后续 Codex 在本目录内工作时，应先读取并遵循该文件。

## 更新记录

- 2026-07-15：使用最新 `imu_utils` Allan 标定结果更新 MRDVS IMU 噪声参数；FAST-LIO2 和 FAST-LIVO2 共 5 个 MRDVS 配置统一采用 `avg-axis` 的 `acc_n=2.0331770033767380e-02`、`gyr_n=3.0946620647727048e-03`、`acc_w=5.4152704615929208e-04`、`gyr_w=4.2000451972629459e-05`。
- 2026-07-10：新增 MRDVS IMU 实时诊断程序，可输出 JSON 汇总、逐样本 CSV 和 PNG 图表；具体实测报告单独归档，不写入项目 README。
- 2026-07-10：完成 FAST-LIVO2 手持稳定性三个代码阶段和 x86_64 软件集成验证：驱动关键模式设置/读回、MRDVS 点与 absolute-us 时间过滤、连续静止 IMU 初始化及 gyro bias 初值均已落地，构建与测试无失败，指定 bag 的抽样点时间跨度低于 `200ms`；真实 MRDVS 快速手持和 RK3588 ARM 原生验收仍未完成。
- 2026-07-10：修复 MRDVS SDK 约每 `2.0~2.5s` 出现重复或小幅回退 IMU 时间戳时，FAST-LIVO2 错误清空静止初始化窗口而长期无法达到 600 个样本的问题；这些非递增样本现在只丢弃且保留初始化进度，前后方向超过 `0.2s` 的大跳变仍会重置同步流，并新增时间戳决策回归测试。
- 2026-07-10：记录 FAST-LIVO2 快速手持扫描稳定性修复设计和测试驱动实施计划：默认采用约 `±500 deg/s` 陀螺量程，显式关闭会破坏逐点时间的 RGBD 对齐和 3D 反畸变，修复近场阈值和异常点时间处理，并采用约 3 秒静止 IMU 初始化；现有 RViz 观察配置保持不变。
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
