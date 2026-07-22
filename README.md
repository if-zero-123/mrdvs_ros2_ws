# mrdvs_ros2_ws

## 工具用途

这是一个 ROS2 工作空间，主要包含 `lx_camera_ros` 包，用于接入蓝芯 MRDVS / Lx Camera SDK，构建相机、障碍物、托盘识别、定位和传感器仿真相关节点；同时集成了 `fastlio2` 主里程计包，用于尝试用 MRDVS 的 LiDAR 点云和 IMU 跑 FAST-LIO2。

## 核心逻辑

- `lx_camera_ros` 通过 `ament_cmake` 构建。
- `CMakeLists.txt` 生成自定义 `msg` 和 `srv` 接口，编译 `lx_camera_node`、`lx_localization_node`、`sensor_sim_node`。
- 相机 SDK 头文件和动态库默认来自 `/opt/MRDVS/include` 与 `/opt/MRDVS/lib`。
- `launch/` 提供相机、雷达、障碍物、托盘、定位、建图、传感器仿真等启动入口。
- 根目录脚本提供常用构建和话题查看命令。
- `fastlio2` 来自 `liangheming/FASTLIO2_ROS2` 的主里程计包；回环作为独立 `pgo` 包接入，本工作空间仍不接入 HBA、localizer 等其他额外模块。
- `fastlio2` 已从 Livox `CustomMsg` 适配为标准 `sensor_msgs/msg/PointCloud2` 输入，读取 MRDVS LiDAR 模式下的 `x/y/z/intensity/timestamp` 字段，并把点时间换算为 FAST-LIO2 去畸变所需的帧内相对毫秒。
- MRDVS IMU SDK 输出单位已经是 `m/s^2` 和 `rad/s`，因此 `fastlio2` 中不再对线加速度额外乘以 10。
- `pgo` 与 `interface` 来自同一上游仓库的提交 `f516daa`；PGO 同步 FAST-LIO2 的机体系点云和局部里程计，沿用位置候选、ICP、GTSAM/iSAM2 流程，通过 `map -> lio_local` 在线修正全局位姿，并提供优化地图保存服务。

## MRDVS 手持采集网页控制台

`tools/mrdvs_collector` 是面向鲁班猫的独立网页采集工具。部署后源码位于 `/home/cat/mrdvs_collector/app`，虚拟环境、配置、运行状态和数据包分别位于同级 `.venv`、`config`、`state` 和 `bags` 目录，不会写入现有 `/home/cat/mrdvs_ros2_ws` ROS 2 underlay。

工具通过 NetworkManager 创建 `MRDVS-Collector` 热点，在 `http://10.42.0.1` 提供手机优先的网页控制台。初始热点密码为 `12345678`，首版不设置额外网页登录。网页可以启动或停止 MRDVS 原始 LiDAR 驱动、填写自定义数据包名称、选择是否随驱动完整录制、查看三维点云与最近 10 秒 IMU、查看日志，以及下载或二次确认删除已经完成的数据包。可用空间低于 5GB 时会优雅停止录制，避免继续写满磁盘。

采集步骤：

1. 手机连接 `MRDVS-Collector`，打开 `http://10.42.0.1`。
2. 在“采集”页填写数据包名称；名称支持中文、英文、数字、短横线和下划线。
3. 需要从驱动启动前开始留存全部数据时，保持“随驱动完整录制全部话题”开启，再点击“启动驱动”。
4. 采集结束后点击“停止驱动”；系统会先停止驱动，再向 rosbag 发送正常停止信号并等待 MCAP 元数据落盘。
5. 在“数据包”页下载 `.tar`，或经过同名二次确认后删除。正在录制的数据包禁止下载和删除。

完整录包使用以下固定命令语义，不抽样、不筛选话题、不改写消息字段或驱动时间戳：

```bash
ros2 bag record --all --include-hidden-topics --storage mcap --output <数据包目录>
```

网页三维点云最多按 5Hz、每帧 50000 点生成独立显示副本，IMU 最多按 20Hz 显示；这些限制只影响浏览器可视化，不会影响 rosbag 进程，因此 `PointCloud2.header.stamp`、点级 `timestamp`、`Imu.header.stamp` 及驱动启动后的其他普通和隐藏话题都会按 ROS 2 原始消息完整写入。

“下次开机自动启动”开关同时控制热点与网页服务。关闭开关只执行 `systemctl disable mrdvs-collector.target`，不会立即关闭当前热点或中断当前采集。热点连接配置固定为 `connection.autoconnect no`，因此下次开机不再启动采集 target 时，NetworkManager 可以自动连接鲁班猫已经保存且允许自动连接的普通 Wi-Fi。需要通过 SSH 恢复热点和网页服务时执行：

```bash
sudo systemctl enable --now mrdvs-collector.target
```

常用服务检查命令：

```bash
systemctl status mrdvs-collector.target mrdvs-hotspot.service mrdvs-web-console.service
journalctl -u mrdvs-web-console.service -n 100 --no-pager
```

## 代码结构

- `src/lx_camera_ros/`：ROS2 功能包源码。
- `src/fastlio2/`：FAST-LIO2 ROS2 主里程计包，已适配 MRDVS 的 `PointCloud2` 和 `Imu` 话题。
- `src/interface/`：上游 FAST-LIO2 扩展模块使用的 ROS2 服务接口，本工作空间的 PGO 使用 `SaveMaps`。
- `src/pgo/`：FAST-LIO2 在线回环和位姿图优化包，包含 MRDVS 配置、独立一体启动、RViz 和契约测试。
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
- `tools/mrdvs_collector/`：独立 FastAPI/WebSocket 采集控制台、离线前端、测试和鲁班猫 systemd/NetworkManager 安装资源。
- `build/`、`install/`、`log/`：ROS2/colcon 生成目录，不应提交到 Git。

## 更新记录

- 2026-07-22：新增鲁班猫 MRDVS 手持采集网页控制台首版，支持热点、原始驱动控制、完整 MCAP 录包、点云/IMU 显示、数据包下载删除、低磁盘保护和下次开机自启设置。

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

### 运行 MRDVS + FAST-LIO2 在线回环

在线回环沿用 `liangheming/FASTLIO2_ROS2` 的 PGO：先按位置和时间搜索历史关键帧，再用 ICP 确认回环，最后由 GTSAM/iSAM2 优化关键帧位姿。PGO 不修改 FAST-LIO2 的局部滤波状态，而是维护以下 TF：

```text
map -> lio_local -> mrdvs_imu -> mrdvs_tof
```

- `map -> lio_local`：PGO 发布，检测到回环后更新全局修正。
- `lio_local -> mrdvs_imu`：FAST-LIO2 发布，保持局部里程计连续。
- `mrdvs_imu -> mrdvs_tof`：现有 FAST-LIO2 launch 发布厂家结构外参。

安装 GTSAM 依赖：

```bash
sudo apt install libgtsam-dev
```

Ubuntu 24.04 的 `libgtsam-dev 4.2.0` 导出文件会引用未随包发布的 `libCppUnitLite.a`；当前 `pgo/CMakeLists.txt` 已绕过损坏的测试库导出，直接使用系统安装的 GTSAM 头文件、共享库和 TBB，不需要手动创建假库或修改 `/usr/lib`。

按当前工作空间兼容方式构建。`lx_camera_ros` 使用普通安装模式，其他三个包使用 symlink install：

```bash
colcon build --packages-select lx_camera_ros \
  --cmake-clean-cache --cmake-args -DBUILD_TESTING=ON
colcon build --packages-select interface fastlio2 pgo \
  --symlink-install --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

启动 MRDVS、FAST-LIO2、PGO 和 PGO RViz：

```bash
source install/setup.bash
ros2 launch pgo mrdvs_pgo_full_launch.py \
  camera_ip:=192.168.100.82 \
  fastlio_delay:=3.0 \
  enable_rviz:=true
```

不需要 RViz 时使用：

```bash
ros2 launch pgo mrdvs_pgo_full_launch.py enable_rviz:=false
```

该入口固定让 FAST-LIO2 加载 `mrdvs_pgo.yaml`，让 PGO 加载 `pgo/config/mrdvs.yaml`；现有无回环入口 `ros2 launch fastlio2 mrdvs_full_launch.py` 不受影响。不要同时启动 FAST-LIVO2，一台 MRDVS 设备不应被两条 SLAM 链路同时占用，也应避免重复发布 TF。

启动后可检查输入和全局修正：

```bash
ros2 topic hz /fastlio2/body_cloud
ros2 topic hz /fastlio2/lio_odom
ros2 topic hz /pgo/optimized_odom
ros2 topic echo --once /pgo/optimized_odom
ros2 topic info /pgo/optimized_path
ros2 topic info /pgo/optimized_map
ros2 topic echo --once /pgo/pose_markers
ros2 run tf2_ros tf2_echo map lio_local
ros2 topic echo /pgo/loop_markers
```

一体启动默认加载独立的 `pgo/rviz/mrdvs_pgo_optimized.rviz`，Fixed Frame 为 `map`。这个 RViz 同时显示：

- `/fastlio2/body_cloud`：当前帧实时扫描，依靠 TF 放到全局坐标系中；不通过长时间 Decay 累积成最终地图。
- `/pgo/optimized_odom`：当前闭环修正后的全局位置和姿态，`frame_id=map`，`child_frame_id=mrdvs_imu`；RViz 用坐标轴显示当前位置，其他 ROS2 节点也可以直接订阅。
- `/pgo/optimized_path`：按 GTSAM 当前结果生成的全部关键帧轨迹，回环后历史轨迹会整体重发。
- `/pgo/optimized_map`：按每个关键帧的 `r_global/t_global` 拼接的历史点云；普通建图时增量追加，接受回环后按全部优化关键帧重建并立即发布。
- `/pgo/pose_markers`：显示固定的 `map` 原点 `(0,0,0)` 标签，以及跟随设备实时更新、保留三位小数的 `x/y/z` 全局位置。
- `/pgo/loop_markers`：被接受的回环节点和连线。

新 RViz 在 `map` 原点和 `mrdvs_imu` 当前位置分别显示红绿蓝坐标轴。默认界面只保留一个 Displays 面板，所有显示项保持折叠，不再恢复之前占用较大空间的 Selection、Tool Properties、Views 和 Time 面板。它不显示 `/fastlio2/world_cloud` 和 `/fastlio2/lio_path`，因为这两个话题属于未闭环修正的局部结果；继续缓存它们会让旧点云留在回环前的位置。默认优化地图使用 `0.1m` 体素，并将普通地图发布限制为最多每 `1.0s` 一次，回环重建不受该限频影响。大场景中如果 CPU 或 DDS 带宽压力明显，可以增大 `pgo/config/mrdvs.yaml` 的 `optimized_map_resolution` 或 `optimized_map_publish_period`。

默认配置每平移 `0.5m` 或旋转 `10deg` 生成关键帧；回环候选需要与当前优化位置相距不超过 `1.0m`，并与当前帧相隔超过 `60s`，ICP fitness score 需要不高于 `0.15`。实测时先静止完成 FAST-LIO2 初始化，再沿闭合路线运行超过 60 秒并回到起点；RViz 中出现 `/pgo/loop_markers` 连线、`map -> lio_local` 从单位变换变为有限修正，表示回环已被接受。未取得闭合路线实测证据前，不要盲目放宽搜索半径或 ICP 阈值。

保存按优化后关键帧拼接的地图时，输出目录必须预先存在：

```bash
OUTPUT_DIR=~/maps/mrdvs_pgo_$(date +%Y%m%d_%H%M%S)
mkdir -p "$OUTPUT_DIR"
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '$OUTPUT_DIR', save_patches: true}"
```

输出包括 `map.pcd`、`poses.txt` 和 `patches/*.pcd`。这些是运行产物，不提交到 Git。在线显示使用 `/pgo/optimized_map`，保存服务则生成未经在线显示体素参数二次降采样的 PCD 和各关键帧位姿，适合后续离线处理。

当前已验证真实设备可以启动驱动、FAST-LIO2、PGO 和新 RViz，`/fastlio2/body_cloud`、`/fastlio2/lio_odom` 与 `/pgo/optimized_odom` 均约为 `10Hz`；实测优化位姿为 `frame_id=map`、`child_frame_id=mrdvs_imu`，与 `map -> mrdvs_imu` TF 数值一致。静止首关键帧已发布 `/pgo/optimized_path` 和包含 3404 点的 `/pgo/optimized_map`，新 RViz 对三个优化话题均已建立订阅，保存地图服务也已验证成功。尚未执行超过 60 秒的真实闭合路线，因此还未验证 MRDVS 场景中的回环检出率、误检率、回环后的历史地图重建画面和闭环误差。另有一个与 PGO 无关的既有驱动问题：`lx_lidar_ros.launch.py` 单独运行时，Ctrl+C 也会在 MRDVS 驱动关闭阶段触发 ROS guard-condition 异常并以 `-6` 退出；进程不会残留，设备可以重新连接。

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

## Codex 工作规则

当前工作空间的 Codex 用户规则写在 `AGENTS.md`。后续 Codex 在本目录内工作时，应先读取并遵循该文件。

## 更新记录

- 2026-07-16：MRDVS LiDAR/SLAM 模式固定并读回校验 `LX_INT_XYZ_COORDINATE=0`；设置、读回失败或值不一致时阻止启动数据流，实机确认 SDK 返回 0 且 XYZIRT 点云正常发布。
- 2026-07-16：PGO RViz 增加 `map` 原点坐标轴、原点 `(0,0,0)` 标签和设备实时 `x/y/z` 数值标签；默认布局精简为单个折叠的 Displays 面板，移除占空间的辅助面板和旧窗口状态。
- 2026-07-15：PGO 新增 `/pgo/optimized_odom`、`/pgo/optimized_path` 和 `/pgo/optimized_map`；普通建图增量拼接关键帧地图，回环后按优化关键帧重建历史地图，并新增独立 RViz 配置同时显示实时扫描、全局位置/姿态、优化轨迹、优化地图和回环连线；实机静止验证优化位姿约 10Hz、首帧优化地图 3404 点，新 RViz 一键启动和话题订阅正常。
- 2026-07-15：从 `liangheming/FASTLIO2_ROS2@f516daa` 接入 `interface` 和在线 PGO，新增 MRDVS 回环专用 `lio_local` 配置和一体启动；真实设备已验证约 `10Hz` 点云/里程计、`map -> lio_local -> mrdvs_imu -> mrdvs_tof` TF 与优化地图保存，超过 60 秒的闭合路线回环验收仍待执行。
- 2026-07-15：使用最新 `imu_utils` Allan 标定结果更新 MRDVS IMU 噪声参数；FAST-LIO2 和 FAST-LIVO2 共 5 个 MRDVS 配置统一采用 `avg-axis` 的 `acc_n=2.0331770033767380e-02`、`gyr_n=3.0946620647727048e-03`、`acc_w=5.4152704615929208e-04`、`gyr_w=4.2000451972629459e-05`。
- 2026-07-10：新增 MRDVS IMU 实时诊断程序，可输出 JSON 汇总、逐样本 CSV 和 PNG 图表；具体实测报告单独归档，不写入项目 README。
- 2026-07-10：完成 FAST-LIVO2 手持稳定性三个代码阶段和 x86_64 软件集成验证：驱动关键模式设置/读回、MRDVS 点与 absolute-us 时间过滤、连续静止 IMU 初始化及 gyro bias 初值均已落地，构建与测试无失败，指定 bag 的抽样点时间跨度低于 `200ms`；真实 MRDVS 快速手持和 RK3588 ARM 原生验收仍未完成。
- 2026-07-10：修复 MRDVS SDK 约每 `2.0~2.5s` 出现重复或小幅回退 IMU 时间戳时，FAST-LIVO2 错误清空静止初始化窗口而长期无法达到 600 个样本的问题；这些非递增样本现在只丢弃且保留初始化进度，前后方向超过 `0.2s` 的大跳变仍会重置同步流，并新增时间戳决策回归测试。
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
