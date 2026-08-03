export const endpoints = Object.freeze({
  status: "/api/status",
  logs: "/api/logs",
  driverStart: "/api/driver/start",
  driverStop: "/api/driver/stop",
  recordingStart: "/api/recording/start",
  recordingStop: "/api/recording/stop",
  bags: "/api/bags",
  settings: "/api/settings",
  hotspot: "/api/settings/hotspot",
  autostart: "/api/settings/autostart",
  pointcloudSocket: "/ws/pointcloud",
  imuSocket: "/ws/imu",
});

export async function request(path, options = {}) {
  const response = await fetch(path, {
    ...options,
    headers: { "Content-Type": "application/json", ...(options.headers || {}) },
  });
  if (!response.ok) {
    const body = await response.json().catch(() => ({ detail: `HTTP ${response.status}` }));
    throw new Error(body.detail || `HTTP ${response.status}`);
  }
  return response.status === 204 ? null : response.json();
}

export function reconnectingWebSocket(path, onMessage, onState) {
  let stopped = false;
  let socket = null;
  const connect = () => {
    const scheme = location.protocol === "https:" ? "wss" : "ws";
    socket = new WebSocket(`${scheme}://${location.host}${path}`);
    socket.binaryType = "arraybuffer";
    socket.onopen = () => onState("connected");
    socket.onmessage = onMessage;
    socket.onerror = () => socket.close();
    socket.onclose = () => {
      onState("offline");
      if (!stopped) window.setTimeout(connect, 1000);
    };
  };
  connect();
  return () => {
    stopped = true;
    if (socket) socket.close();
  };
}

export const getStatus = () => request(endpoints.status);
export const getLogs = () => request(endpoints.logs);
export const getBags = () => request(endpoints.bags);
export const getSettings = () => request(endpoints.settings);
export const startDriver = (record, bagName, options = {}) => request(endpoints.driverStart, {
  method: "POST",
  body: JSON.stringify({
    record,
    bag_name: bagName || null,
    rgb_mode: options.rgb_mode || "raw",
    topic_mode: options.topic_mode || "all",
    selected_topics: options.selected_topics || null,
  }),
});
export const stopDriver = () => request(endpoints.driverStop, { method: "POST" });
export const startRecording = (bagName, options = {}) => request(endpoints.recordingStart, {
  method: "POST",
  body: JSON.stringify({
    bag_name: bagName,
    rgb_mode: options.rgb_mode || "raw",
    topic_mode: options.topic_mode || "all",
    selected_topics: options.selected_topics || null,
  }),
});
export const stopRecording = () => request(endpoints.recordingStop, { method: "POST" });
export const deleteBag = (name, confirmation) => request(`${endpoints.bags}/${encodeURIComponent(name)}`, { method: "DELETE", body: JSON.stringify({ confirmation }) });
export const downloadBagUrl = (name) => `${endpoints.bags}/${encodeURIComponent(name)}/download`;
export const saveSettings = (settings) => request(endpoints.settings, { method: "PUT", body: JSON.stringify(settings) });
export const saveHotspot = (settings) => request(endpoints.hotspot, { method: "PUT", body: JSON.stringify(settings) });
export const saveAutostart = (enabled) => request(endpoints.autostart, { method: "PUT", body: JSON.stringify({ enabled }) });
