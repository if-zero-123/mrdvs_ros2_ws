# MRDVS 原始/压缩 RGB 与选择性话题录制设计

## 1. 目标

在 MRDVS 手持采集网页中增加可配置的录制范围，使操作者可以在完整录制全部 MRDVS 话题和仅录制预置话题之间选择，并可以选择 RGB 原始图像或 ROS `compressed` 图像。采集必须保留驱动消息自身的时间戳，不因网页显示限频、JPEG 编码或话题筛选而改变其他传感器数据。

## 2. 已确认的用户选择

- RGB 录制方式只有两个互斥选项：原始图像、压缩图像。
- 压缩图像使用 JPEG quality 100，保持原始分辨率和驱动帧率。
- JPEG quality 100 仍属于有损编码；需要逐像素一致时使用原始图像模式。
- 页面刷新或鲁班猫重启后，录制方式默认恢复为原始图像，不记住上次的压缩选择。
- 录制范围只有两个选项：全部 MRDVS 话题、选择话题。
- 选择话题模式只显示预置的 MRDVS 话题，不允许手动输入任意话题名。
- `/tf` 和 `/tf_static` 在选择话题模式下始终录制。
- 压缩节点启动失败或运行中异常退出时，停止 rosbag，并把数据包标记为错误。

## 3. 方案与架构

采用独立的 `image_transport` 压缩发布节点，不修改厂商驱动内部的 RGB publisher：

```text
/lx_camera_node/LxCamera_Rgb (sensor_msgs/Image)
        ├── 原始模式：rosbag 直接录制
        └── image_transport republish
                └── /lx_camera_node/LxCamera_Rgb/compressed
                        (sensor_msgs/CompressedImage)
```

压缩节点由采集控制器按录制方式启动和停止。它订阅原始图像并发布 JPEG；压缩消息复制原始 `header.stamp` 和 `frame_id`，只增加编码处理延迟，不重新生成采集时间戳。压缩过程与网页 RGB 预览完全独立，预览的 5Hz 限制不能影响录包。

## 4. 预置话题清单

网页以分组复选框显示以下 MRDVS 话题：

| 分组 | 话题 |
| --- | --- |
| RGB | `/lx_camera_node/LxCamera_Rgb`、`/lx_camera_node/LxCamera_Rgb/compressed`、`/lx_camera_node/LxCamera_RgbInfo` |
| 三维 | `/lx_camera_node/LxCamera_Cloud`、`/lx_camera_node/LxCamera_Depth`、`/lx_camera_node/LxCamera_Amp`、`/lx_camera_node/LxCamera_TofInfo` |
| 惯导 | `/lx_camera_node/LxCamera_Imu` |
| 诊断与算法 | `/lx_camera_node/LxCamera_Error`、`/lx_camera_node/LxCamera_Message`、`/lx_camera_node/LxCamera_FrameRate`、`/lx_camera_node/LxCamera_Obstacle`、`/lx_camera_node/LxCamera_Pallet` |
| 坐标变换 | `/lx_camera_node/LxCamera_TF` |

`/tf` 和 `/tf_static` 不显示为可取消复选框，在选择话题模式下自动加入。原始 RGB 与 compressed RGB 互斥：选择原始模式时仅允许原始话题，选择压缩模式时仅允许 compressed 话题。

默认范围为“全部 MRDVS 话题”，默认 RGB 方式为“原始图像”。

## 5. 录包命令与启动顺序

控制器在驱动启动前启动 rosbag，以便录到驱动刚开始发布的数据。录包启动成功后再启动驱动；压缩模式还必须在 rosbag 前启动并确认压缩发布节点已存活。

- 全部模式：使用完整预置话题列表（不使用网页显示订阅），并追加 `/tf`、`/tf_static`。
- 选择模式：使用用户勾选的话题列表，并追加固定的 `/tf`、`/tf_static`。
- 原始 RGB：不启动压缩节点，录制 `/lx_camera_node/LxCamera_Rgb`。
- compressed RGB：启动压缩节点，录制 `/lx_camera_node/LxCamera_Rgb/compressed`，排除原始 `/lx_camera_node/LxCamera_Rgb`。

所有录制模式都使用 MCAP 存储；不抽样、不降帧、不改写消息字段。rosbag 的写入时间只属于 MCAP 记录元数据，回放和多传感器同步仍使用消息内部的 `header.stamp`。

## 6. 网页交互

采集页新增两个控件：

1. RGB 录制方式：原始图像 / 压缩图像（JPEG 100）。
2. 录制范围：全部 MRDVS 话题 / 选择话题。

选择“选择话题”后显示预置话题分组和勾选状态。开始录制前，后端返回最终解析的话题列表和 RGB 方式，页面在运行状态卡片中显示，防止操作者误以为网页预览设置决定录包内容。默认每次打开页面或重启服务都恢复原始图像和全部话题。

## 7. 完整性与故障处理

- 压缩发布节点无法启动、启动后立即退出、运行中退出或 rosbag 订阅建立失败时，控制器停止驱动和 rosbag，并把数据包置为错误状态。
- 停止流程为：停止驱动，停止压缩节点（如有），向 rosbag 发送 SIGINT，等待进程退出和 MCAP 元数据落盘。
- 压缩节点的队列和监控必须有界；检测到无法维持压缩发布时宁可失败并提示，也不能静默生成缺少 RGB 帧的“完成”数据包。
- 原始驱动、点云、IMU、深度和其他已选话题不依赖网页浏览器连接；浏览器断开不会停止录制。

## 8. 测试要求

自动测试应覆盖：

- 两种 RGB 方式生成正确的原始或 compressed 话题集合，且不会同时录制两种 RGB；
- 全部模式和选择模式都会保留 `/tf`、`/tf_static`；
- 预置话题清单、去重、非法组合和空选择能够得到明确错误；
- 压缩消息继承原始 `header.stamp`、`frame_id`、宽高，并使用 JPEG quality 100；
- rosbag 在驱动前启动，压缩节点异常会停止采集并标记数据包错误；
- 既有完整录包、驱动生命周期、网页预览、下载和磁盘空间保护测试不回归。

## 9. 部署边界

实现完成后通过 USB 有线 SSH 部署到鲁班猫 `/home/cat/mrdvs_collector`。部署前确认没有活动驱动或 rosbag；只同步应用和必要的 MRDVS 驱动变更，保留远端 `bags/`、`config/` 和 `state/`。先在本机运行单元测试，再在鲁班猫进行短时启动验证，确认原始/压缩选择、话题集合和异常停止行为。

