# MRDVS 原始 RGB 采集与网页预览设计

## 1. 目标

在手持采集网页控制台启动 MRDVS LiDAR 驱动时，明确关闭 SDK 的 2D RGB 去畸变，保证 `/lx_camera_node/LxCamera_Rgb` 是保留镜头畸变的原始驱动图像。同时在网页“实时数据”页增加低带宽 RGB 预览，且预览处理不得改变、抽样或阻塞 rosbag 中的原始 RGB 消息。

## 2. 范围

本次包含：

- `lx_lidar_ros.launch.py` 强制设置 `LX_BOOL_ENABLE_2D_UNDISTORT=0`；
- RGB 显示桥、状态统计和 `/ws/rgb` WebSocket；
- 移动端 RGB 预览卡片；
- 本机与鲁班猫测试、USB 有线 SSH 部署和短时实机预览验证。

本次不包含：

- RGBD 对齐、3D 去畸变或点云运动补偿；
- 在浏览器传输原始 BGR 数据；
- 对 rosbag 中的 RGB 消息进行 JPEG 压缩、改写或抽样；
- Depth、Amp 图像预览；
- 视频录制、视频转码或历史帧缓存。

## 3. 驱动设置

网页仍只启动：

```bash
ros2 launch lx_camera_ros lx_lidar_ros.launch.py \
  ip:=192.168.100.82 \
  enable_rviz:=false \
  imu_angular_range_level:=<配置值>
```

LiDAR launch 增加固定参数：

```python
{"LX_BOOL_ENABLE_2D_UNDISTORT": 0}
```

驱动现有 `SET_INT_PARAM` 逻辑会因此调用 SDK：

```text
DcSetBoolValue(handle, LX_BOOL_ENABLE_2D_UNDISTORT, false)
```

`LX_INT_RGBD_ALIGN_MODE=0` 和 `LX_BOOL_ENABLE_3D_UNDISTORT=0` 保持不变。2D 去畸变缩放参数在关闭状态下不设置。启动验证必须确认 ROS 参数值为 `0`，且驱动日志没有 SDK 设置错误。

## 4. RGB 显示数据流

```text
/lx_camera_node/LxCamera_Rgb
        ├── ros2 bag record --all：原始 ROS Image，完整写入
        └── RosBridge：最新帧显示副本
                ├── 原始到达频率统计
                ├── 有网页客户端时最高 5Hz 编码
                ├── 最长边超过 1280 时等比例缩小
                ├── JPEG 质量 80
                └── /ws/rgb → 浏览器最新帧
```

ROS bridge 使用 sensor-data QoS 订阅 RGB。支持驱动实际使用的 `bgr8`、`rgb8` 和 `mono8`，通过 ROS 的 `cv_bridge` 与 OpenCV 完成颜色转换、缩放和 JPEG 编码。不能识别的编码只更新预览错误状态，不影响驱动或录包。

桥只保存一个最新 JPEG 字节串、序号、尺寸和接收时间，不保存历史图像。WebSocket 客户端数量为零时不执行 JPEG 编码，只保留话题频率和最后到达时间统计；多个客户端复用同一份编码结果。

显示参数写入 `AppConfig`，默认值与约束为：

```text
rgb_max_hz = 5.0，范围 (0, 20]
rgb_max_dimension = 1280，范围 [160, 3840]
rgb_jpeg_quality = 80，范围 [30, 95]
```

首版不在网页设置页开放这三个参数，避免扩大交互范围；后续可通过既有配置模型扩展。

## 5. API 与断连处理

新增 `/ws/rgb`。每次发送的二进制消息是一个完整 JPEG 文件，不自定义二进制头；浏览器可直接构造 `image/jpeg` Blob。连接过程继续使用既有 Origin 白名单。

WebSocket 采用最新值语义：慢客户端不会形成无界队列，新帧覆盖旧帧。连接建立和断开分别增加、减少 RGB 查看者计数；断开监听与点云/IMU 一致，浏览器关闭后端点必须及时退出。服务关闭时不应残留 WebSocket、编码或 ROS 线程。

`/api/status` 的 topics 增加 RGB 原始到达频率、最后消息年龄和预览错误。这里的 RGB Hz 与点云/IMU 顶部状态一致，表示 bridge 接收到的原始话题频率，不是 5Hz 显示帧率。

## 6. 网页界面

“实时数据”页在点云卡片前增加 RGB 卡片，内容包括：

- 最新 RGB 图像，保持宽高比，容器内完整显示；
- WebSocket 连接状态；
- 实际显示 FPS；
- JPEG 分辨率；
- 无数据和编码失败提示。

浏览器每次收到新 JPEG 后创建新的 Blob URL，并立即释放上一帧 URL，防止长期运行时内存增长。页面隐藏或网络断开后使用现有自动重连机制，不向后端累计待发送帧。

## 7. 完整性与故障隔离

RGB 预览与 rosbag 完全分支。录包命令保持：

```bash
ros2 bag record --all --include-hidden-topics --storage mcap
```

JPEG 缩放、限频、客户端断开或浏览器卡顿都不能修改 ROS 消息、停止驱动、改变 `/lx_camera_node/LxCamera_Rgb` 发布频率，或向 rosbag 施加背压。原始 RGB 的 header、编码、宽高、步长和数据字段由 rosbag2 直接序列化。

## 8. 测试

自动测试覆盖：

- LiDAR launch 明确包含 `LX_BOOL_ENABLE_2D_UNDISTORT=0`，并保持 RGBD 对齐和 3D 去畸变关闭；
- RGB 限频在统计原始频率之后执行；
- 等比例缩放最长边不超过 1280；
- JPEG 可以解码，尺寸和颜色通道正确；
- 无查看者时不编码，多查看者复用最新帧；
- `/ws/rgb` 发送 JPEG、限制 Origin、客户端关闭后及时退出；
- 前端释放旧 Blob URL，显示 FPS 与尺寸；
- 既有 83 项采集控制台测试和 ROS 包测试不回归；
- `bag_command` 仍是完整录制命令。

## 9. 部署与实机验证

部署前通过 USB 有线地址 `cat@192.168.100.100` 检查没有活动驱动或 rosbag。只同步本次涉及的 `lx_camera_ros` 和 `tools/mrdvs_collector` 文件，保留远端数据包、配置和状态目录。

鲁班猫上执行：

- 重新构建并安装 `lx_camera_ros`；
- 重新安装 `mrdvs-web-console` Python 包；
- 重启网页服务，不切换 USB 网络配置；
- 运行板端测试和 launch 参数解析。

短时实机验证只启动原始驱动，不录制数据包：确认 SDK 2D 去畸变参数为 `0`、原始 RGB 话题正常、网页能收到最高 5Hz 的 JPEG、点云和 IMU 仍正常。验证后停止驱动并确认无残留进程。

## 10. Git 边界

用户已有的 `README.md` 和 `docs/images/` 改动保持不动。本功能的设计、实现、测试和部署提交只选择性暂存相关文件；不使用自动全量暂存。
