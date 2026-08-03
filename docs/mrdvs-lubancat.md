# MRDVS 鲁班猫部署与采集指南

本文档专门说明如何在鲁班猫上部署和使用 MRDVS 手持数据网页采集程序。项目总体结构和开发规则请返回查看[主 README](../README.md)。

## 部署结构与网络

网页程序与 ROS 2 工作空间分开部署，避免运行数据污染驱动源码：

```text
/home/cat/mrdvs_collector/
├── app/       # 从本仓库 tools/mrdvs_collector 同步的应用源码
├── .venv/     # Python 虚拟环境，通过 system-site-packages 使用 rclpy
├── config/    # 持久化设置
├── state/     # 数据包状态等运行状态
└── bags/      # 默认 MCAP 数据包根目录

/home/cat/mrdvs_ros2_ws/
└── install/   # 当前 MRDVS 驱动 ROS 2 underlay
```

默认网络如下：

| 用途 | 接口或地址 | 说明 |
| --- | --- | --- |
| 手机热点 | `wlan0` / `10.42.0.1` | SSID `MRDVS-Collector`，初始密码 `12345678` |
| USB 网口 SSH/导出 | `eth1` / `192.168.100.100` | 电脑端当前使用 `192.168.100.99` |
| 板载设备网口 | `eth0` / `192.168.100.66` | 当前 MRDVS 设备地址为 `192.168.100.82` |
| 网页 | `http://10.42.0.1` | 仅监听热点地址的 80 端口 |

## 首次部署或更新

先通过 USB 网口把应用同步到独立目录，不复制本机的虚拟环境、Node.js 依赖或缓存：

```bash
ssh cat@192.168.100.100 'mkdir -p /home/cat/mrdvs_collector/app'

rsync -avh --delete \
  --exclude='.venv/' \
  --exclude='node_modules/' \
  --exclude='.pytest_cache/' \
  --exclude='__pycache__/' \
  --exclude='*.egg-info/' \
  tools/mrdvs_collector/ \
  cat@192.168.100.100:/home/cat/mrdvs_collector/app/
```

在鲁班猫上执行幂等安装器：

```bash
ssh -t cat@192.168.100.100
cd /home/cat/mrdvs_collector/app
bash deploy/install_lubancat.sh
```

压缩 RGB 录制依赖 ROS 2 的 `image_transport` 和 `compressed_image_transport`。安装器会在修改系统前检查这两个包；如果提示缺少 compressed 插件，可先通过鲁班猫的 ROS 软件源安装：

```bash
sudo apt-get install ros-jazzy-compressed-image-transport
```

安装器创建 ARM64 虚拟环境、安装 Python 包、校验 sudoers、安装 systemd 单元、创建热点配置并启用下次开机自启；它不会在安装过程中立即切换当前网络。更新应用后可执行：

```bash
sudo systemctl restart mrdvs-web-console.service
```

## 开机、自启动和访问网页

正常情况下，`mrdvs-collector.target` 会在开机时同时启动热点和网页：

1. 给鲁班猫和传感器上电，等待系统启动。
2. 手机连接 `MRDVS-Collector`。
3. 输入热点密码 `12345678`。
4. 浏览器打开 `http://10.42.0.1`。
5. 页面右上角显示“服务在线”后再开始操作。

通过 USB 网口检查服务：

```bash
ssh cat@192.168.100.100
systemctl status \
  mrdvs-collector.target \
  mrdvs-hotspot.service \
  mrdvs-web-console.service
```

需要恢复并同时设置下次开机自启：

```bash
sudo systemctl enable --now mrdvs-collector.target
```

只启动本次、不改变下次开机设置：

```bash
sudo systemctl start mrdvs-collector.target
```

设置页中的“下次开机自动启动热点和网页”只控制下一次启动：关闭开关不会立即停止当前服务，也不会中断当前采集；关闭后鲁班猫仍可以恢复连接已经保存并允许自动连接的普通 Wi-Fi。

## 网页采集流程

1. 打开“采集”页，在“数据包名称”中填写本次任务名称。名称允许中文、英文字母、数字、短横线和下划线，最长 80 个字符；禁止路径分隔符、`.`、`..`、控制字符和同名覆盖。
2. 选择 RGB 录制方式：默认“原始图像”；也可以选择“压缩图像（JPEG 质量 100）”。JPEG 质量 100 仍是有损编码，需要逐像素一致时选择原始图像。
3. 选择录制范围：默认“全部 MRDVS 话题”；选择“选择话题”后可在预置清单中勾选，`/tf` 和 `/tf_static` 始终保留。RGB 原始和 compressed 话题互斥。
4. 需要从驱动启动时刻完整留存数据时，勾选“随驱动启动录制”。
5. 点击“启动驱动”。后端会先启动压缩节点（如选择 compressed），再启动 rosbag，最后启动设备驱动，避免遗漏驱动最初发布的话题。
6. 确认驱动和录制状态均正常；根据需要查看实时数据和日志。
7. 采集结束点击“停止驱动”。后端按驱动、压缩节点、rosbag 顺序停止，并等待 MCAP 元数据写完。
8. 进入“数据包”页检查名称、大小、状态和时长。

如果只启动了驱动，也可以在驱动运行期间填写包名、选择录制方式和话题后单独点击“开始录制”和“停止录制”。压缩节点或 rosbag 异常退出时，数据包会自动停止并标记为错误。

## 录包完整性与网页显示

默认“全部 MRDVS 话题 + 原始图像”模式使用：

```bash
ros2 bag record \
  --all \
  --include-hidden-topics \
  --storage mcap \
  --output <安全的数据包目录>
```

该命令不抽样、不修改消息字段，也不改写驱动时间戳。compressed 模式会启动独立的：

```bash
ros2 run image_transport republish raw compressed \
  --ros-args \
  -r in:=/lx_camera_node/LxCamera_Rgb \
  -r out:=/lx_camera_node/LxCamera_Rgb \
  -p out.compressed.jpeg_quality:=100
```

压缩模式录制 `/lx_camera_node/LxCamera_Rgb/compressed`，排除原始 RGB；其他已选话题保持完整。压缩消息复制原始 RGB 的 `header.stamp` 和 `frame_id`，压缩延迟不会改写采集时间戳。网页点云最高 5FPS、点数最多 50000，IMU 曲线最高 20Hz，这些显示限制都不会影响 MCAP 完整录制。

## 数据包保存、下载和 USB 导出

默认保存目录：

```text
/home/cat/mrdvs_collector/bags/<网页填写的数据包名称>/
```

大包推荐通过 USB 网口使用 rsync：

```bash
mkdir -p ~/MRDVS_bags

rsync -avhs --partial --info=progress2 \
  cat@192.168.100.100:/home/cat/mrdvs_collector/bags/ \
  ~/MRDVS_bags/
```

只导出指定数据包：

```bash
rsync -avhs --partial --info=progress2 \
  'cat@192.168.100.100:/home/cat/mrdvs_collector/bags/数据包名称/' \
  ~/MRDVS_bags/数据包名称/
```

正在录制的数据包禁止下载和删除。网页删除需要输入完整数据包名称二次确认；删除后不可恢复。

## 离线核对传感器时间和录包时间

鲁班猫上的只读分析命令：

```bash
source /opt/ros/jazzy/setup.bash
/home/cat/mrdvs_collector/.venv/bin/mrdvs-timestamp-analysis \
  /home/cat/mrdvs_collector/bags/<数据包名称>
```

分析会分别保留 `PointCloud2.header.stamp` / `Imu.header.stamp`、点级 `timestamp` 和 MCAP 接收时间。帧级时间用于传感器同步，点级时间用于点云帧内去畸变，MCAP 接收时间只用于排查录制积压，不能替代传感器时间。

## 设置、自启和恢复

“设置”页可以修改雷达 IP、IMU 量程、数据包根目录、热点名称和密码。热点名称或密码修改后在下一次热点启动时应用。

网页正常但需要单独重启：

```bash
sudo systemctl restart mrdvs-web-console.service
journalctl -u mrdvs-web-console.service -n 100 --no-pager
```

## 常见问题

| 现象 | 检查和处理 |
| --- | --- |
| 手机找不到热点 | USB SSH 后检查 `systemctl status mrdvs-hotspot.service`，必要时启动 `mrdvs-collector.target` |
| 热点能连接但网页打不开 | 检查 `mrdvs-web-console.service` 和 `ss -ltnp \| grep ':80 '` |
| 点击启动后驱动变为异常 | 查看网页日志；在鲁班猫上手动执行对应 `ros2 launch ... --show-args` 检查包、launch 和参数 |
| 点云 FPS 低于原始 Hz | 正常，网页点云最高 5FPS；完整 rosbag 不限频 |
| 录制自动停止 | 检查剩余空间；低于 5GB 时系统会主动停止录制 |
| 数据包不能下载或删除 | 正在录制的数据包受保护，先正常停止录制 |
| USB SSH 不通 | 电脑检查 `192.168.100.99/24`，鲁班猫 USB 网口地址为 `192.168.100.100/24` |

