# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Browser-side WASDQE speed control; Viser has no public API for it."""

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
