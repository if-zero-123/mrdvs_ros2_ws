import * as THREE from "/static/vendor/three.module.min.js";

export function decodePointCloud(buffer) {
  const view = new DataView(buffer);
  if (view.byteLength < 8) throw new Error("点云帧长度不足");
  const magic = String.fromCharCode(...new Uint8Array(buffer, 0, 4));
  if (magic !== "MPC1") throw new Error("点云帧标识错误");
  const count = view.getUint32(4, true);
  if (view.byteLength !== 8 + count * 16) throw new Error("点云帧长度与点数不匹配");
  return { count, values: new Float32Array(buffer, 8, count * 4) };
}

export class PointCloudViewer {
  constructor(container, onStats) {
    this.container = container;
    this.onStats = onStats;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x030912);
    this.camera = new THREE.PerspectiveCamera(58, 1, 0.02, 200);
    this.camera.position.set(0, 0, 5);
    this.renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: "high-performance" });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.container.appendChild(this.renderer.domElement);
    this.geometry = new THREE.BufferGeometry();
    this.material = new THREE.PointsMaterial({ size: 0.025, vertexColors: true, sizeAttenuation: true });
    this.points = new THREE.Points(this.geometry, this.material);
    this.scene.add(this.points);
    this.drag = null;
    this.lastFrameTime = performance.now();
    this.bindInput();
    this.observer = new ResizeObserver(() => this.resize());
    this.observer.observe(container);
    this.resize();
    this.render();
  }

  update(buffer) {
    const { count, values } = decodePointCloud(buffer);
    const positions = new Float32Array(count * 3);
    const colors = new Float32Array(count * 3);
    let minIntensity = Infinity;
    let maxIntensity = -Infinity;
    for (let index = 0; index < count; index += 1) {
      const intensity = values[index * 4 + 3];
      minIntensity = Math.min(minIntensity, intensity);
      maxIntensity = Math.max(maxIntensity, intensity);
    }
    const span = Math.max(maxIntensity - minIntensity, 1);
    const color = new THREE.Color();
    for (let index = 0; index < count; index += 1) {
      positions.set(values.subarray(index * 4, index * 4 + 3), index * 3);
      const level = (values[index * 4 + 3] - minIntensity) / span;
      color.setHSL(0.55 - level * 0.5, 0.9, 0.58);
      colors[index * 3] = color.r;
      colors[index * 3 + 1] = color.g;
      colors[index * 3 + 2] = color.b;
    }
    this.geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
    this.geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
    this.geometry.computeBoundingSphere();
    const now = performance.now();
    const fps = 1000 / Math.max(now - this.lastFrameTime, 1);
    this.lastFrameTime = now;
    this.onStats({ count, fps });
  }

  bindInput() {
    const canvas = this.renderer.domElement;
    canvas.addEventListener("pointerdown", (event) => { this.drag = { x: event.clientX, y: event.clientY }; canvas.setPointerCapture(event.pointerId); });
    canvas.addEventListener("pointermove", (event) => {
      if (!this.drag) return;
      this.points.rotation.y += (event.clientX - this.drag.x) * 0.006;
      this.points.rotation.x += (event.clientY - this.drag.y) * 0.006;
      this.drag = { x: event.clientX, y: event.clientY };
    });
    canvas.addEventListener("pointerup", () => { this.drag = null; });
    canvas.addEventListener("wheel", (event) => {
      event.preventDefault();
      this.camera.position.z = THREE.MathUtils.clamp(this.camera.position.z + event.deltaY * 0.004, 0.3, 40);
    }, { passive: false });
  }

  reset() {
    this.points.rotation.set(0, 0, 0);
    this.camera.position.set(0, 0, 5);
  }

  resize() {
    const width = Math.max(this.container.clientWidth, 1);
    const height = Math.max(this.container.clientHeight, 1);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height, false);
  }

  render() {
    this.animation = requestAnimationFrame(() => this.render());
    this.renderer.render(this.scene, this.camera);
  }

  destroy() {
    cancelAnimationFrame(this.animation);
    this.observer.disconnect();
    this.geometry.dispose();
    this.material.dispose();
    this.renderer.dispose();
  }
}
