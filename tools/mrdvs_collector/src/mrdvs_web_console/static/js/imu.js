const colors = ["#35d7e5", "#4ee1a0", "#ffb454"];

function makeDatasets(labels) {
  return labels.map((label, index) => ({ label, data: [], borderColor: colors[index], borderWidth: 1.5, pointRadius: 0, tension: 0.16 }));
}

function chartOptions(title) {
  return {
    responsive: true,
    maintainAspectRatio: false,
    animation: false,
    parsing: false,
    normalized: true,
    plugins: { legend: { labels: { color: "#a8bdd1" } }, title: { display: true, text: title, color: "#edf6ff" } },
    scales: {
      x: { type: "linear", ticks: { color: "#7890a8", callback: (value) => `${Number(value).toFixed(1)}s` }, grid: { color: "rgba(150,180,210,.08)" } },
      y: { ticks: { color: "#7890a8" }, grid: { color: "rgba(150,180,210,.08)" } },
    },
  };
}

export class ImuViewer {
  constructor(accelerationCanvas, angularCanvas, currentElement) {
    this.currentElement = currentElement;
    this.acceleration = new window.Chart(accelerationCanvas, { type: "line", data: { datasets: makeDatasets(["Ax", "Ay", "Az"]) }, options: chartOptions("线加速度 m/s²") });
    this.angular = new window.Chart(angularCanvas, { type: "line", data: { datasets: makeDatasets(["Gx", "Gy", "Gz"]) }, options: chartOptions("角速度 rad/s") });
  }

  update(sample) {
    const stamp = Number(sample.stamp);
    this.append(this.acceleration, stamp, sample.linear_acceleration);
    this.append(this.angular, stamp, sample.angular_velocity);
    const acc = sample.linear_acceleration;
    const gyr = sample.angular_velocity;
    this.currentElement.textContent = `A ${acc.x.toFixed(2)}, ${acc.y.toFixed(2)}, ${acc.z.toFixed(2)} · G ${gyr.x.toFixed(2)}, ${gyr.y.toFixed(2)}, ${gyr.z.toFixed(2)}`;
  }

  append(chart, stamp, vector) {
    const cutoff = stamp - 10;
    [vector.x, vector.y, vector.z].forEach((value, index) => {
      const data = chart.data.datasets[index].data;
      data.push({ x: stamp, y: value });
      while (data.length && data[0].x < cutoff) data.shift();
    });
    chart.update("none");
  }
}
