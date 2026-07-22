import {
  deleteBag, downloadBagUrl, endpoints, getBags, getLogs, getSettings, getStatus,
  reconnectingWebSocket, saveAutostart, saveHotspot, saveSettings, startDriver,
  startRecording, stopDriver, stopRecording,
} from "/static/js/api.js";
import { ImuViewer } from "/static/js/imu.js";
import { PointCloudViewer } from "/static/js/pointcloud.js";

const byId = (id) => document.getElementById(id);
const connection = byId("connection-state");
const bagName = byId("bag-name");
const recordWithDriver = byId("record-with-driver");
const messageLine = byId("session-message");
const toast = byId("toast");
let currentStatus = null;
let currentSettings = null;
let deleteTarget = null;

const pointCloud = new PointCloudViewer(byId("pointcloud-view"), ({ count, fps }) => {
  byId("point-count").textContent = `${count.toLocaleString()} 点 · ${fps.toFixed(1)} FPS`;
});
const imu = new ImuViewer(byId("acceleration-chart"), byId("angular-chart"), byId("imu-current"));

function showToast(text, error = false) {
  toast.textContent = text;
  toast.className = `toast show${error ? " error" : ""}`;
  window.clearTimeout(showToast.timer);
  showToast.timer = window.setTimeout(() => { toast.className = "toast"; }, 3200);
}

function requireBagName() {
  const value = bagName.value.trim();
  if (!value) throw new Error("请先填写数据包名称");
  return value;
}

function stateText(value) {
  return ({ stopped: "已停止", starting: "启动中", running: "运行中", stopping: "停止中", error: "异常", recording: "录制中" })[value] || value || "未知";
}

function formatBytes(value) {
  if (!Number.isFinite(value)) return "--";
  const units = ["B", "KB", "MB", "GB", "TB"];
  let amount = value;
  let unit = 0;
  while (amount >= 1024 && unit < units.length - 1) { amount /= 1024; unit += 1; }
  return `${amount.toFixed(unit >= 3 ? 1 : 0)} ${units[unit]}`;
}

function updateStatus(status) {
  currentStatus = status;
  byId("driver-status").textContent = stateText(status.driver_state);
  byId("recording-status").textContent = stateText(status.recording_state);
  byId("cloud-status").textContent = `${Number(status.topics.cloud_hz).toFixed(1)} Hz`;
  byId("imu-status").textContent = `${Number(status.topics.imu_hz).toFixed(1)} Hz`;
  byId("disk-status").textContent = `${formatBytes(status.disk.free_bytes)} 可用`;
  byId("active-bag").textContent = status.active_bag_name || "未录制";
  const driverBusy = ["starting", "stopping"].includes(status.driver_state);
  const recordingBusy = ["starting", "stopping"].includes(status.recording_state);
  byId("start-driver").disabled = status.driver_state === "running" || driverBusy;
  byId("stop-driver").disabled = status.driver_state !== "running" || driverBusy;
  byId("start-recording").disabled = status.driver_state !== "running" || status.recording_state === "recording" || recordingBusy;
  byId("stop-recording").disabled = status.recording_state !== "recording" || recordingBusy;
  bagName.disabled = status.recording_state === "recording" || recordingBusy;
  recordWithDriver.disabled = status.driver_state === "running" || driverBusy;
  messageLine.textContent = status.last_error || status.last_warning || "";
}

async function refreshStatus() {
  try {
    const status = await getStatus();
    updateStatus(status);
    connection.dataset.state = "connected";
    connection.lastElementChild.textContent = "服务在线";
  } catch (error) {
    connection.dataset.state = "offline";
    connection.lastElementChild.textContent = "连接中断";
    messageLine.textContent = error.message;
  }
}

async function refreshLogs() {
  try { byId("log-view").textContent = (await getLogs()).lines.join("\n") || "暂无日志"; }
  catch (error) { showToast(error.message, true); }
}

async function refreshBags() {
  try {
    const bags = await getBags();
    const list = byId("bag-list");
    list.replaceChildren();
    if (!bags.length) { list.innerHTML = '<p class="empty-state">还没有数据包</p>'; return; }
    bags.forEach((bag) => {
      const row = document.createElement("article");
      row.className = "bag-row";
      const info = document.createElement("div");
      info.className = "bag-info";
      const title = document.createElement("strong");
      title.textContent = bag.name;
      const detail = document.createElement("small");
      detail.textContent = `${formatBytes(bag.size_bytes)} · ${stateText(bag.state)} · ${bag.duration_seconds == null ? "时长未知" : `${Number(bag.duration_seconds).toFixed(1)}s`}`;
      info.append(title, detail);
      const download = document.createElement("a");
      download.href = downloadBagUrl(bag.name);
      download.className = "button-link";
      download.textContent = "下载";
      download.setAttribute("download", "");
      const remove = document.createElement("button");
      remove.className = "danger compact";
      remove.textContent = "删除";
      remove.disabled = bag.state === "recording";
      remove.addEventListener("click", () => openDeleteDialog(bag.name));
      row.append(info, download, remove);
      list.append(row);
    });
  } catch (error) { showToast(error.message, true); }
}

async function loadSettings() {
  try {
    currentSettings = await getSettings();
    byId("hotspot-ssid").value = currentSettings.hotspot_ssid || "MRDVS-Collector";
    byId("hotspot-password").value = "";
    byId("radar-ip").value = currentSettings.radar_ip;
    byId("imu-range").value = currentSettings.imu_range_level;
    byId("bag-root").value = currentSettings.bag_root;
    byId("autostart-enabled").checked = currentSettings.autostart_enabled;
  } catch (error) { showToast(error.message, true); }
}

function openDeleteDialog(name) {
  deleteTarget = name;
  byId("delete-target").textContent = name;
  byId("delete-confirmation").value = "";
  byId("delete-dialog").showModal();
}

document.querySelectorAll(".tab-button").forEach((button) => button.addEventListener("click", () => {
  document.querySelectorAll(".tab-button").forEach((item) => item.classList.toggle("active", item === button));
  document.querySelectorAll(".tab-panel").forEach((panel) => panel.classList.toggle("active", panel.dataset.panel === button.dataset.tab));
  if (button.dataset.tab === "bags") refreshBags();
  if (button.dataset.tab === "settings") loadSettings();
}));

byId("start-driver").addEventListener("click", async () => {
  try { await startDriver(recordWithDriver.checked, recordWithDriver.checked ? requireBagName() : null); await refreshStatus(); }
  catch (error) { showToast(error.message, true); }
});
byId("stop-driver").addEventListener("click", async () => { try { await stopDriver(); await refreshStatus(); await refreshBags(); } catch (error) { showToast(error.message, true); } });
byId("start-recording").addEventListener("click", async () => { try { await startRecording(requireBagName()); await refreshStatus(); } catch (error) { showToast(error.message, true); } });
byId("stop-recording").addEventListener("click", async () => { try { await stopRecording(); await refreshStatus(); await refreshBags(); } catch (error) { showToast(error.message, true); } });
byId("refresh-logs").addEventListener("click", refreshLogs);
byId("refresh-bags").addEventListener("click", refreshBags);
byId("reset-view").addEventListener("click", () => pointCloud.reset());

byId("confirm-delete").addEventListener("click", async (event) => {
  event.preventDefault();
  try {
    await deleteBag(deleteTarget, byId("delete-confirmation").value);
    byId("delete-dialog").close();
    await refreshBags();
    showToast("数据包已删除");
  } catch (error) { showToast(error.message, true); }
});

byId("settings-form").addEventListener("submit", async (event) => {
  event.preventDefault();
  try {
    const autostart = byId("autostart-enabled").checked;
    if (currentSettings && currentSettings.autostart_enabled && !autostart) {
      const recovery = "sudo systemctl enable --now mrdvs-collector.target";
      if (!window.confirm(`关闭后，下次开机不会启动热点和网页。恢复命令：\n${recovery}\n\n确认关闭下次开机自启？`)) return;
    }
    await saveSettings({
      radar_ip: byId("radar-ip").value.trim(),
      imu_range_level: Number(byId("imu-range").value),
      bag_root: byId("bag-root").value.trim(),
    });
    const password = byId("hotspot-password").value;
    if (password || byId("hotspot-ssid").value !== currentSettings.hotspot_ssid) {
      if (!password) throw new Error("修改热点名称时必须同时填写新密码");
      await saveHotspot({ ssid: byId("hotspot-ssid").value.trim(), password });
    }
    if (!currentSettings || autostart !== currentSettings.autostart_enabled) await saveAutostart(autostart);
    await loadSettings();
    showToast("设置已保存");
  } catch (error) { showToast(error.message, true); }
});

reconnectingWebSocket(endpoints.pointcloudSocket, (event) => pointCloud.update(event.data), () => {});
reconnectingWebSocket(endpoints.imuSocket, (event) => imu.update(JSON.parse(event.data)), () => {});
window.addEventListener("beforeunload", () => pointCloud.destroy());

refreshStatus();
refreshLogs();
refreshBags();
window.setInterval(refreshStatus, 1000);
window.setInterval(refreshLogs, 3000);
window.setInterval(refreshBags, 5000);
