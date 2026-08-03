# MRDVS Selective Topic Recording Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task with TDD checkpoints.

**Goal:** 在 MRDVS 网页采集器中加入原始/压缩 RGB 二选一、全部/预置话题选择两种录制范围，并保证 rosbag、压缩节点和驱动的完整性与异常处理。

**Architecture:** 采集控制器负责解析录制选项、构造显式 rosbag 话题列表和管理独立 `image_transport republish` 进程。原始模式直接录制 `sensor_msgs/Image`，compressed 模式启动 JPEG quality 100 的压缩发布节点并只录制 `sensor_msgs/CompressedImage`；网页仅负责提交选择和展示最终录制配置。

**Tech Stack:** Python 3.12、FastAPI、Pydantic v2、asyncio、ROS 2 Jazzy `ros2 bag`、`image_transport`、MCAP、原生 HTML/CSS/ES modules、pytest/pytest-asyncio。

## Global Constraints

- RGB 录制方式只有 `raw` 和 `compressed` 两种，默认 `raw`；compressed 使用 JPEG quality 100。
- compressed 消息必须保留原始消息的 `header.stamp` 和 `frame_id`，不使用网页 5Hz 预览副本。
- 录制范围只有 `all` 和 `selected` 两种；selected 只允许预置 MRDVS 话题，不接受任意文本话题名。
- `/tf`、`/tf_static` 在 selected 模式中始终加入；RGB raw 与 compressed 互斥。
- rosbag 在驱动前启动；compressed 模式须先确认压缩进程存活；压缩进程或 rosbag 异常时数据包标记为错误。
- 停止顺序固定为驱动、压缩进程、rosbag，并等待 MCAP 元数据落盘。
- 不修改用户已有的 `README.md`、`tools/mrdvs_collector/pyproject.toml`、`timestamp_analysis.py` 及其测试改动。
- 每个任务先写失败测试，测试通过后单独提交；提交只暂存本任务文件。

## 文件与职责映射

- Modify `tools/mrdvs_collector/src/mrdvs_web_console/controller.py`: 录制枚举、预置话题解析、bag/压缩命令、三进程生命周期和异常监控。
- Modify `tools/mrdvs_collector/src/mrdvs_web_console/schemas.py`: API 请求中的 RGB 方式、录制范围和预置话题 ID 校验。
- Modify `tools/mrdvs_collector/src/mrdvs_web_console/api.py`: 把录制选项传入控制器，并在状态响应中返回最终解析的录制配置。
- Modify `tools/mrdvs_collector/src/mrdvs_web_console/static/index.html`: 增加 RGB 方式、录制范围和预置话题复选框。
- Modify `tools/mrdvs_collector/src/mrdvs_web_console/static/js/api.js`: 扩展启动驱动和开始录制请求参数。
- Modify `tools/mrdvs_collector/src/mrdvs_web_console/static/js/app.js`: 默认值、互斥逻辑、选择话题交互和运行状态展示。
- Modify `tools/mrdvs_collector/src/mrdvs_web_console/static/styles.css`: 话题选择分组的移动端布局。
- Test `tools/mrdvs_collector/tests/test_controller.py`: 命令构造和三进程生命周期。
- Test `tools/mrdvs_collector/tests/test_api.py`: 请求校验、状态回显和错误响应。
- Test `tools/mrdvs_collector/tests/test_static_contract.py`: 前端控件和默认值静态契约。

---

### Task 1: 建立录制选项模型、预置话题和命令工厂

**Files:**
- Modify: `tools/mrdvs_collector/src/mrdvs_web_console/controller.py`
- Modify: `tools/mrdvs_collector/src/mrdvs_web_console/schemas.py`
- Test: `tools/mrdvs_collector/tests/test_controller.py`

**Interfaces:**
- `RgbRecordingMode(str, Enum)`: `RAW="raw"`, `COMPRESSED="compressed"`。
- `TopicRecordingMode(str, Enum)`: `ALL="all"`, `SELECTED="selected"`。
- `PRESET_TOPICS: tuple[str, ...]`：包含设计文档中的 13 个 MRDVS 基础话题，不含 `/tf`、`/tf_static` 和 compressed 派生话题。
- `FIXED_TOPICS: tuple[str, ...] = ("/tf", "/tf_static")`。
- `resolve_topics(rgb_mode, topic_mode, selected_topics) -> tuple[str, ...]`：去重、按预置顺序排序、追加固定 TF；selected 为空或包含未知话题时抛出 `CollectorConflict`。
- `bag_command(path: Path, topics: tuple[str, ...] | None = None) -> list[str]`：`topics is None` 保留现有 `--all --include-hidden-topics` 命令；显式话题使用 `--topics` 和同样的 MCAP/output 参数。
- `compression_command() -> list[str]`：返回 `ros2 run image_transport republish raw compressed`，把 `in` 和 `out` 基础话题映射到 `/lx_camera_node/LxCamera_Rgb`，并通过 ROS 参数设置 JPEG quality 100。

- [ ] **Step 1: 写失败测试**

在 `test_controller.py` 增加以下断言：

```python
def test_selected_topics_add_tf_and_replace_rgb():
    result = resolve_topics(
        RgbRecordingMode.COMPRESSED,
        TopicRecordingMode.SELECTED,
        ("/lx_camera_node/LxCamera_Rgb", "/lx_camera_node/LxCamera_Imu"),
    )
    assert result == (
        "/lx_camera_node/LxCamera_Rgb/compressed",
        "/lx_camera_node/LxCamera_Imu",
        "/tf",
        "/tf_static",
    )

def test_bag_command_for_explicit_topics(tmp_path):
    command = bag_command(tmp_path / "bags" / "room1", ("/topic_a", "/tf"))
    assert command == [
        "ros2", "bag", "record", "--topics", "/topic_a", "/tf",
        "--include-hidden-topics", "--storage", "mcap", "--output",
        str(tmp_path / "bags" / "room1"),
    ]

def test_compression_command_sets_quality_100():
    command = compression_command()
    assert command[:6] == ["ros2", "run", "image_transport", "republish", "raw", "compressed"]
    assert "in:=/lx_camera_node/LxCamera_Rgb" in command
    assert "out:=/lx_camera_node/LxCamera_Rgb" in command
    assert "jpeg_quality:=100" in command
```

- [ ] **Step 2: 运行失败测试**

运行：

```bash
cd tools/mrdvs_collector
pytest -q tests/test_controller.py -k 'selected_topics or bag_command_for_explicit or compression_command'
```

预期：FAIL，提示新枚举、解析函数或命令工厂不存在。

- [ ] **Step 3: 实现最小模型和命令工厂**

在 `controller.py` 顶部增加枚举和常量；`resolve_topics` 对 raw/compressed 只替换 RGB 主话题，对 selected 只接受 `PRESET_TOPICS`，最后按预置顺序、固定 TF 顺序去重。扩展 `schemas.py` 的 `DriverStartRequest` 和 `RecordingStartRequest`，字段使用 `RgbRecordingMode`、`TopicRecordingMode` 和 `selected_topics: list[str] | None`，默认 raw/all，并拒绝 selected 空列表。

- [ ] **Step 4: 运行测试确认通过**

运行同一 pytest 命令，预期新增测试全部 PASS，旧的 `test_exact_command_factories` 也 PASS。

- [ ] **Step 5: 提交**

```bash
git add tools/mrdvs_collector/src/mrdvs_web_console/controller.py tools/mrdvs_collector/src/mrdvs_web_console/schemas.py tools/mrdvs_collector/tests/test_controller.py
git commit -m "feat: add selective recording topic model"
```

### Task 2: 扩展控制器管理 rosbag、压缩节点和错误状态

**Files:**
- Modify: `tools/mrdvs_collector/src/mrdvs_web_console/controller.py`
- Test: `tools/mrdvs_collector/tests/test_controller.py`

**Interfaces:**
- `CollectorController.start_driver(record, bag_name, rgb_mode=RAW, topic_mode=ALL, selected_topics=None)`。
- `CollectorController.start_recording(bag_name, rgb_mode=RAW, topic_mode=ALL, selected_topics=None)`。
- `RuntimeSnapshot` 新增 `rgb_recording_mode`, `topic_recording_mode`, `recorded_topics`, `compression_returncode`。
- `ProcessRunner.start` 的进程名固定使用 `"compression"`、`"rosbag"`、`"driver"`，便于日志和监控。

- [ ] **Step 1: 写失败测试**

扩展 `FakeProcess`/`FakeRunner`，增加以下测试：

```python
@pytest.mark.asyncio
async def test_compressed_recording_starts_compression_rosbag_then_driver(tmp_path):
    controller, runner, _ = make_controller(tmp_path)
    await controller.start_driver(
        record=True, bag_name="compressed", rgb_mode=RgbRecordingMode.COMPRESSED
    )
    assert [name for name, _ in runner.commands] == ["compression", "rosbag", "driver"]
    assert "/lx_camera_node/LxCamera_Rgb/compressed" in runner.commands[1][1]
    assert "/lx_camera_node/LxCamera_Rgb" not in runner.commands[1][1]
    await controller.shutdown()

@pytest.mark.asyncio
async def test_compression_exit_marks_bag_error_and_stops_rosbag(tmp_path):
    controller, runner, bags = make_controller(tmp_path)
    await controller.start_driver(
        record=True, bag_name="broken", rgb_mode=RgbRecordingMode.COMPRESSED
    )
    runner.processes["compression"].exit(9)
    await asyncio.sleep(0)
    await asyncio.sleep(0)
    assert controller.snapshot().recording_state is RecordingState.ERROR
    assert bags._load_status("broken")["state"] is BagState.ERROR
    assert runner.stop_order == ["rosbag"]
    await controller.shutdown()

@pytest.mark.asyncio
async def test_stop_order_is_driver_compression_rosbag(tmp_path):
    controller, runner, _ = make_controller(tmp_path)
    await controller.start_driver(record=True, bag_name="room", rgb_mode=RgbRecordingMode.COMPRESSED)
    await controller.stop_driver()
    assert runner.stop_order == ["driver", "compression", "rosbag"]
    await controller.shutdown()
```

- [ ] **Step 2: 运行失败测试**

运行：`pytest -q tests/test_controller.py -k 'compressed_recording or compression_exit or stop_order_is_driver_compression'`。

预期：FAIL，当前控制器只管理 driver/rosbag。

- [ ] **Step 3: 实现最小生命周期改造**

在控制器中增加 `_compression`、`_compression_state` 和选项字段；compressed 录制时先启动压缩进程并执行启动宽限检查，再启动 rosbag，最后启动驱动。原始模式跳过压缩进程。停止和 shutdown 按驱动、压缩、rosbag 顺序退出；`_monitor_compression` 与现有 rosbag 监控一样，在非主动停止退出时调用 `_stop_recording_locked(BagState.ERROR, "RGB 压缩节点异常退出，录制已停止")` 并停止驱动。启动任一子进程失败时，按已启动的逆生命周期清理并标记 bag error。

- [ ] **Step 4: 运行测试确认通过**

运行完整控制器测试：`pytest -q tests/test_controller.py`，预期全部 PASS。

- [ ] **Step 5: 提交**

```bash
git add tools/mrdvs_collector/src/mrdvs_web_console/controller.py tools/mrdvs_collector/tests/test_controller.py
git commit -m "feat: manage compressed recording process lifecycle"
```

### Task 3: 接入 API 参数、状态回显和严格校验

**Files:**
- Modify: `tools/mrdvs_collector/src/mrdvs_web_console/api.py`
- Modify: `tools/mrdvs_collector/tests/test_api.py`

**Interfaces:**
- `POST /api/driver/start` 接受 `record`, `bag_name`, `rgb_mode`, `topic_mode`, `selected_topics`。
- `POST /api/recording/start` 接受同样的 RGB/话题字段。
- `GET /api/status` 返回控制器快照中的 `rgb_recording_mode`, `topic_recording_mode`, `recorded_topics`, `compression_returncode`。

- [ ] **Step 1: 写失败测试**

在 API 测试中提交 compressed + selected 请求，断言 FakeController 收到完整参数；再提交 selected 空列表和未知话题，断言 HTTP 400；断言 status JSON 返回话题列表和模式。

- [ ] **Step 2: 运行失败测试**

运行：`pytest -q tests/test_api.py -k 'recording_mode or selected_topics or status'`，预期 FAIL。

- [ ] **Step 3: 实现路由转发和错误信息**

扩展路由调用，把 Pydantic 枚举和选中的字符串列表传入控制器；保留现有 400/409 异常处理；`asdict(snapshot)` 自动包含新增字段。

- [ ] **Step 4: 运行 API 测试**

运行：`pytest -q tests/test_api.py`，预期全部 PASS。

- [ ] **Step 5: 提交**

```bash
git add tools/mrdvs_collector/src/mrdvs_web_console/api.py tools/mrdvs_collector/tests/test_api.py
git commit -m "feat: expose recording selection through API"
```

### Task 4: 完成采集页的录制方式和预置话题选择

**Files:**
- Modify: `tools/mrdvs_collector/src/mrdvs_web_console/static/index.html`
- Modify: `tools/mrdvs_collector/src/mrdvs_web_console/static/js/api.js`
- Modify: `tools/mrdvs_collector/src/mrdvs_web_console/static/js/app.js`
- Modify: `tools/mrdvs_collector/src/mrdvs_web_console/static/styles.css`
- Test: `tools/mrdvs_collector/tests/test_static_contract.py`

**Interfaces:**
- `startDriver(record, bagName, rgbMode, topicMode, selectedTopics)` 和 `startRecording(bagName, rgbMode, topicMode, selectedTopics)` JSON 字段与 API 完全一致。
- DOM IDs：`rgb-recording-mode`, `topic-recording-mode`, `topic-selection`, `topic-selection-list`, `recorded-topics`。

- [ ] **Step 1: 写失败静态契约测试**

断言 index.html 包含两个 select、话题分组复选框容器、`JPEG 质量 100` 文案和固定 TF 文案；断言 app.js 设置 `raw`/`all` 默认值，并把 `selected_topics` 发送到两个启动请求。

- [ ] **Step 2: 运行失败测试**

运行：`pytest -q tests/test_static_contract.py -k 'recording or topic'`，预期 FAIL。

- [ ] **Step 3: 实现 HTML、JS 和 CSS**

在采集卡片中加入 RGB 方式下拉框和录制范围下拉框；选择 selected 时显示按设计文档分组的固定复选框，默认全部勾选；切换 compressed 时自动取消 raw RGB 复选框并勾选 compressed，反向切换同理；禁止用户取消最后一个 RGB 方式。`requireBagName` 保持不变。开始驱动/开始录制时收集勾选的预置话题并提交；状态刷新把后端 `recorded_topics` 渲染到 `recorded-topics`。

- [ ] **Step 4: 运行静态测试并检查语法**

运行：`pytest -q tests/test_static_contract.py`；再执行 `node --check tools/mrdvs_collector/src/mrdvs_web_console/static/js/app.js` 和 `node --check tools/mrdvs_collector/src/mrdvs_web_console/static/js/api.js`，预期均成功。

- [ ] **Step 5: 提交**

```bash
git add tools/mrdvs_collector/src/mrdvs_web_console/static/index.html tools/mrdvs_collector/src/mrdvs_web_console/static/js/api.js tools/mrdvs_collector/src/mrdvs_web_console/static/js/app.js tools/mrdvs_collector/src/mrdvs_web_console/static/styles.css tools/mrdvs_collector/tests/test_static_contract.py
git commit -m "feat: add web recording topic selectors"
```

### Task 5: 完成全量回归、安装契约和部署验证

**Files:**
- Modify: `tools/mrdvs_collector/tests/test_deployment_contract.py` only if the new ROS executable dependency needs an explicit contract.
- Modify: `tools/mrdvs_collector/tests/test_packaging_contract.py` only if package metadata needs a runtime dependency check.
- Modify: `README.md` only after the implementation is verified, and only in the MRDVS collector usage section.

- [ ] **Step 1: 写失败部署契约测试**

增加断言：安装/部署脚本不会把 `bags/`、`config/`、`state/` 删除；运行环境检查包含 `image_transport` executable 或对应 ROS package；服务重启命令保持现有名称。

- [ ] **Step 2: 运行失败测试**

运行：`pytest -q tests/test_deployment_contract.py tests/test_packaging_contract.py`，预期新依赖契约先失败。

- [ ] **Step 3: 更新安装与使用说明**

确认鲁班猫 underlay 已安装 `image_transport` compressed plugin；若未安装，在安装器中加入 ROS Jazzy 包安装/校验，不写入 sudo 密码。README 仅补充页面字段、两种录制范围、压缩质量 100、默认 raw 和异常行为。

- [ ] **Step 4: 运行完整本地验证**

```bash
cd tools/mrdvs_collector
pytest -q
python -m compileall -q src
```

预期：全部 pytest 通过、Python 编译无输出。使用 `ros2 run image_transport republish --help` 确认板端命令可执行；若命令不存在，安装对应 ROS Jazzy image transport 包后重试。

- [ ] **Step 5: 提交回归与文档**

```bash
git add tools/mrdvs_collector/tests/test_deployment_contract.py tools/mrdvs_collector/tests/test_packaging_contract.py README.md
git commit -m "docs: document selective MRDVS recording"
```

- [ ] **Step 6: 通过 USB SSH 部署并验证**

部署前执行 `ssh cat@192.168.100.100 'pgrep -af "lx_lidar|ros2 bag record|image_transport republish" || true'`，确认没有活动采集进程；同步应用，执行鲁班猫安装器，重启 `mrdvs-web-console.service`。短时验证 raw/all、compressed/selected 各启动一次，不下载或删除用户已有数据包；确认 `/tf`、`/tf_static`、选择话题和 JPEG 100 生效，最后确认无残留进程。

- [ ] **Step 7: 创建快照**

```bash
git tag "snapshot-$(date +%Y%m%d-%H%M)"
git push origin feature/mrdvs-web-console --follow-tags
```

部署验证完成后报告 tag、提交和任何板端风险。

## Self-review checklist

- 设计文档第 2、4、5、7、8、9 节分别由 Task 1–5 覆盖。
- 所有新增命名在任务接口中固定：`RgbRecordingMode`、`TopicRecordingMode`、`resolve_topics`、`compression_command` 和快照字段一致。
- 未引入自定义话题输入、同时 raw+compressed 模式或网页预览复用，符合已确认范围。
- 每个任务均包含失败测试、运行命令、最小实现、通过测试和独立提交。
