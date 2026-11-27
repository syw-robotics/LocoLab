# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Browser-side camera controls and MP4 recording of the Viser scene."""

CAMERA_MOVE_SPEED_JS = """
window.__locolabMoveSpeed = __MOVE_SPEED__;
if (!window.__locolabCameraSpeedInstalled) {
  window.__locolabCameraSpeedInstalled = true;
  const held = new Set();
  const keys = {
    truck: ["KeyA", "KeyD"],
    forward: ["KeyW", "KeyS"],
    elevate: ["KeyQ", "KeyE"],
  };
  function findMutable() {
    if (window.__viserMutable) return window.__viserMutable;
    // Older Viser versions keep the viewer ref in React hooks only.
    const root = document.getElementById("root");
    const key = root && Object.keys(root).find(name => name.startsWith("__reactContainer"));
    const fiber = key && root[key];
    const stack = fiber ? [fiber.current || fiber] : [];
    while (stack.length) {
      const node = stack.pop();
      for (let hook = node.memoizedState; hook; hook = hook.next) {
        const value = hook.memoizedState?.current;
        if (value?.cameraControl) return value;
      }
      if (node.child) stack.push(node.child);
      if (node.sibling) stack.push(node.sibling);
    }
    return null;
  }
  function wrapCamera() {
    const mutable = findMutable();
    const controls = mutable?.cameraControl;
    if (!controls || controls.__locolabSpeedWrapped) return;
    const methods = new Map();
    // Wrap the reference, rather than overwriting methods: Viser may cache
    // its own method wrappers to request rendering after camera movement.
    mutable.cameraControl = new Proxy(controls, {
      get(target, name) {
        if (name === "__locolabSpeedWrapped") return true;
        const value = Reflect.get(target, name, target);
        if (typeof value !== "function") return value;
        if (!methods.has(name)) {
          methods.set(name, (...args) => {
            if (keys[name]?.some(key => held.has(key)) && args.at(-1) === false) {
              const scale = window.__locolabMoveSpeed / 2.0;
              args[0] *= scale;
              if (name === "truck") args[1] *= scale;
            }
            return value.apply(target, args);
          });
        }
        return methods.get(name);
      },
    });
  }
  document.addEventListener("keydown", event => {
    if (!Object.values(keys).some(codes => codes.includes(event.code))) return;
    if (event.target.closest?.("input, textarea, select, [contenteditable]")) return;
    held.add(event.code);
    wrapCamera();
  }, true);
  document.addEventListener("keyup", event => held.delete(event.code), true);
  window.addEventListener("blur", () => held.clear());
}
"""

VIDEO_RECORDING_JS = """
const startId = "__START_ID__";
const stopId = "__STOP_ID__";
const root = document.documentElement;
let style = document.getElementById("locolab-video-controls");
const installed = Boolean(style);
if (!style) {
  style = document.createElement("style");
  style.id = "locolab-video-controls";
  document.head.append(style);
}
style.dataset.start = startId;
style.dataset.stop = stopId;
style.dataset.task = __TASK_NAME__;
style.textContent = `
  :root:not([data-recording-video]) div:has(> button[id="${stopId}"]),
  :root[data-recording-video] div:has(> button[id="${startId}"]) { display: none; }
`;
if (installed) return;

let recorder;
document.addEventListener("click", event => {
  const button = event.target.closest?.("button");
  if (!button || (button.id !== style.dataset.start && button.id !== style.dataset.stop)) return;
  event.stopPropagation();
  if (button.id === style.dataset.stop) {
    if (recorder?.state === "recording") recorder.stop();
    return;
  }
  if (recorder) return;
  const canvas = window.__viserMutable?.canvas ?? document.querySelector('#root canvas[data-engine^="three.js"]');
  if (!canvas?.captureStream || !window.MediaRecorder?.isTypeSupported("video/mp4")) {
    window.alert("Video recording requires a loaded 3D scene and a browser with MP4 canvas recording support.");
    return;
  }

  const task = style.dataset.task.split(":").at(-1).replace(/[^a-zA-Z0-9_-]+/g, "-").slice(0, 100) || "viser";
  const mode = document.querySelector("[data-locolab-mode]")?.dataset.locolabMode ?? "live";
  const now = new Date();
  const localTime = new Date(now.getTime() - now.getTimezoneOffset() * 60000);
  const stamp = localTime.toISOString().slice(0, -1).replace("T", "_").replace(/:/g, "-");
  const filename = `${task}_${mode}_${stamp}.mp4`;

  let stream;
  let timer;
  try {
    // The WebGL canvas is transparent where the viewer's CSS supplies the sky.
    // Composite just that background and the 3D canvas into the recording.
    const output = document.createElement("canvas");
    output.width = canvas.width;
    output.height = canvas.height;
    const ctx = output.getContext("2d", { alpha: false });
    let background;
    for (let node = canvas; node; node = node.parentElement) {
      const computed = getComputedStyle(node);
      if (computed.backgroundColor !== "rgba(0, 0, 0, 0)") {
        background = computed;
        break;
      }
    }
    const draw = () => {
      ctx.fillStyle = background?.backgroundColor ?? "#fff";
      ctx.fillRect(0, 0, output.width, output.height);
      const scale = Math.min(output.width / canvas.width, output.height / canvas.height);
      const width = canvas.width * scale;
      const height = canvas.height * scale;
      ctx.drawImage(canvas, (output.width - width) / 2, (output.height - height) / 2, width, height);
    };
    draw();
    stream = output.captureStream();
    recorder = new MediaRecorder(stream, { mimeType: "video/mp4" });
    const chunks = [];
    recorder.ondataavailable = ({ data }) => { if (data.size) chunks.push(data); };
    recorder.onerror = ({ error }) => window.alert("Video recording failed: " + error.message);
    recorder.onstop = () => {
      clearInterval(timer);
      stream.getTracks().forEach(track => track.stop());
      recorder = null;
      root.removeAttribute("data-recording-video");
      if (!chunks.length) return;
      const url = URL.createObjectURL(new Blob(chunks, { type: "video/mp4" }));
      const link = document.createElement("a");
      link.href = url;
      link.download = filename;
      link.click();
      setTimeout(() => URL.revokeObjectURL(url), 60000);
    };
    recorder.start(1000);
    timer = setInterval(draw, 1000 / 30);
    root.setAttribute("data-recording-video", "");
  } catch (error) {
    clearInterval(timer);
    stream?.getTracks().forEach(track => track.stop());
    recorder = null;
    window.alert("Could not record the 3D scene: " + error.message);
  }
}, true);
"""
