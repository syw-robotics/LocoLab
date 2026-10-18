# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Code injected into the Viser page, and the websocket knobs around it.

Viser does not expose keyboard speed or a way to run script on connect, so the
viewer installs one script and widens the broadcast buffer directly.
"""

from __future__ import annotations

import viser

# Installed once in the browser. Viser does not expose the keyboard speed, so
# this scales the camera-controls methods that WASDQE already call.
VIEWER_KEYS_JS = """
if (!window.__locolabViewerKeys) {
  window.__locolabViewerKeys = true;
  window.__locolabMoveSpeed = window.__locolabMoveSpeed || __MOVE_SPEED__;
  const baseSpeed = 2;
  function controlsFrom(value) {
    if (!value) return null;
    if (typeof value.truck === "function") return value;
    if (value.current && typeof value.current.truck === "function") return value.current;
    if (value.current && value.current.cameraControl && typeof value.current.cameraControl.truck === "function") {
      return value.current.cameraControl;
    }
    if (typeof value.getState === "function") {
      try {
        const state = value.getState();
        if (state && typeof state.controls?.truck === "function") return state.controls;
      } catch (error) {}
    }
    return null;
  }
  function findControls() {
    const canvases = document.querySelectorAll("canvas");
    for (const canvas of canvases) {
      const root = canvas.__r3f && canvas.__r3f.root;
      const controls = root && controlsFrom(root);
      if (controls) return controls;
    }
    const host = document.getElementById("root");
    if (!host) return null;
    const key = Object.keys(host).find(
      (name) => name.startsWith("__reactContainer") || name.startsWith("__reactFiber")
    );
    let fiber = key ? host[key] : null;
    if (fiber && fiber.current) fiber = fiber.current;
    const seen = new Set();
    const stack = fiber ? [fiber] : [];
    while (stack.length) {
      const node = stack.pop();
      if (!node || seen.has(node)) continue;
      seen.add(node);
      let hook = node.memoizedState;
      while (hook) {
        const controls = controlsFrom(hook.memoizedState);
        if (controls) return controls;
        hook = hook.next;
      }
      if (node.child) stack.push(node.child);
      if (node.sibling) stack.push(node.sibling);
    }
    return null;
  }
  function wrapControls(controls) {
    if (controls.__locolabWrapped) return;
    const scale = () => (window.__locolabMoveSpeed || baseSpeed) / baseSpeed;
    const wrap = (name) => {
      const original = controls[name].bind(controls);
      controls[name] = function (distance, second, third) {
        const factor = scale();
        if (name === "truck") return original(distance * factor, second * factor, third);
        return original(distance * factor, second);
      };
    };
    wrap("truck");
    wrap("forward");
    wrap("elevate");
    controls.__locolabWrapped = true;
    window.__locolabControlsWrapped = true;
  }
  const timer = setInterval(() => {
    const controls = findControls();
    if (!controls) return;
    wrapControls(controls);
    clearInterval(timer);
  }, 200);
  function clickLabel(names) {
    for (const button of document.querySelectorAll("button")) {
      const text = (button.innerText || "").trim();
      if (names.indexOf(text) !== -1) {
        button.click();
        return;
      }
    }
  }
  function toggleCheckbox(label) {
    const nodes = document.querySelectorAll("p, span, label");
    for (const node of nodes) {
      if (node.childElementCount > 0) continue;
      if ((node.textContent || "").trim() !== label) continue;
      const row = node.parentElement && node.parentElement.parentElement;
      const input = row && row.querySelector("input[type='checkbox']");
      if (input) {
        input.click();
        return;
      }
    }
  }
  document.addEventListener("keydown", (event) => {
    const tag = event.target && event.target.tagName;
    if (tag === "INPUT" || tag === "TEXTAREA" || event.repeat) return;
    if (event.code === "Space") {
      event.preventDefault();
      clickLabel(["Pause", "Play"]);
    } else if (event.code === "KeyR") {
      event.preventDefault();
      clickLabel(["Reset"]);
    } else if (event.code === "KeyF") {
      event.preventDefault();
      toggleCheckbox("Track env");
    }
  });
}
"""


def run_javascript(server: viser.ViserServer, source: str) -> None:
    """Run source in every connected browser, including clients that join later."""
    from viser import _messages

    server.gui._websock_interface.queue_message(_messages.RunJavascriptMessage(source=source))


def widen_broadcast_window(server: viser.ViserServer) -> None:
    """Send one pose refresh in a single websocket frame when Viser allows it.

    The default window is 128 messages. Several robots with a dozen moving
    joints exceed that, so the browser applies a pose in pieces. The buffer is
    private; if a future Viser moves it, the viewer still works.
    """
    buffer = getattr(getattr(server, "_websock_server", None), "_broadcast_buffer", None)
    if buffer is not None and hasattr(buffer, "max_window_size"):
        buffer.max_window_size = max(int(buffer.max_window_size), 8192)
