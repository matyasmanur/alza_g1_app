import { createViewer } from "/static/viewer.js?v=3";

const $ = id => document.getElementById(id);
const DEG = 180 / Math.PI;
const rows = [];
const pending = new Map();
const commands = new Map();
const featured = { "recorded-face-wave": "👋", "recorded-high-five": "🖐️", "recorded-handshake": "🤝" };
const clock = () => new Date().toLocaleTimeString("en-GB");
let socket, state, viewer;
let nextId = 0;
let side = "left";
let backendLabel = "";
let readOnly = false;
let locomotion = false;
let recording = false;
let recordedPoses = 0;
let recorderPending = false;
let lastStateAt = 0;
let lastHeartbeat = 0;
let nextSliderWrite = 0;
let sliderTimer = null;

const messages = {
  take_control: "Control acquired. Enable motion when ready.",
  enable: "Motion enabled. Adjust an arm joint.",
  set_joint: "Joint target accepted.",
  gesture: "Gesture started.",
  ready: "Moving to the Ready pose.",
  release_control: "Releasing control.",
  set_mode: "Changing control mode. Wait for the robot to settle.",
  stop: "Stop requested. Waiting for the arms to settle.",
  reset_stop: "Stop reset. Motion is disabled.",
};

const driveButtons = [...document.querySelectorAll("[data-drive]")];
const modeButtons = [...document.querySelectorAll("[data-control-mode]")];
const held = new Map();
const driveKeys = {
  KeyW: [1, 0, 0], KeyS: [-1, 0, 0], KeyA: [0, 1, 0],
  KeyD: [0, -1, 0], KeyQ: [0, 0, 1], KeyE: [0, 0, -1],
};
const driveLimits = [0.35, 0.25, 0.5];

function canDrive() {
  return state?.connection === "connected" && state.mode === "locomotion" &&
    state.owned && state.enabled && !state.switching && !state.stop_latched;
}

function sendDrive() {
  const active = [...new Set(held.values())];
  const axes = [0, 0, 0];
  for (const key of active) driveKeys[key].forEach((value, index) => { axes[index] += value; });
  const planar = Math.max(1, Math.hypot(axes[0], axes[1]));
  const velocity = axes.map((value, index) => value * driveLimits[index] / (index < 2 ? planar : 1));
  for (const button of driveButtons) button.toggleAttribute("data-held", active.includes(button.dataset.drive));
  if (!active.length || canDrive() && socket?.bufferedAmount < 1024) send("drive", { velocity });
}

function clearDrive() {
  if (!held.size) return;
  held.clear();
  sendDrive();
}

function releaseDrive(source) {
  if (held.delete(source)) sendDrive();
}

function feedback(text, error = false) {
  const status = $("command-status");
  if (status.textContent !== text) {
    const earlier = document.createElement("li");
    earlier.textContent = status.textContent;
    earlier.dataset.time = status.dataset.time;
    earlier.dataset.tone = status.dataset.tone;
    $("log").prepend(earlier);
    $("log").children[4]?.remove();
    status.textContent = text;
    status.dataset.time = clock();
    // shortcut: colliding links are read from the clearance message; send them as a field if its wording changes.
    const clash = text.match(/between (.+) and (.+?)\./);
    viewer?.highlight("clash", clash ? clash.slice(1).map(name => name.replaceAll(" ", "_")) : []);
  }
  status.dataset.tone = error ? "danger" : "neutral";
}
$("command-status").dataset.time = clock();

function send(type, values = {}) {
  if (socket?.readyState !== WebSocket.OPEN) return;
  const id = ++nextId;
  if (type !== "heartbeat" && type !== "drive") commands.set(id, [type, performance.now()]);
  socket.send(JSON.stringify({ type, id, ...values }));
}

function angleText(angle) {
  if (!Number.isFinite(angle)) return "—";
  const degrees = angle * DEG;
  return `${(Math.abs(degrees) < 0.05 ? 0 : degrees).toFixed(1)}°`;
}

function flushTargets() {
  if (!pending.size || sliderTimer !== null) return;
  const wait = Math.max(0, nextSliderWrite - performance.now());
  const congested = socket?.bufferedAmount > 1024;
  if (wait || congested) {
    sliderTimer = setTimeout(() => {
      sliderTimer = null;
      flushTargets();
    }, congested ? Math.max(33, wait) : wait);
    return;
  }
  nextSliderWrite = performance.now() + 33;
  for (const [joint, angle] of pending) send("set_joint", { joint, angle });
  pending.clear();
}

function addRow(joint, index) {
  const root = document.createElement("div");
  root.className = "joint";
  root.innerHTML = `<div class="slider"><div><label></label><small></small></div><input type="range" step="0.1" disabled></div>
    <span class="angle target">—</span><span class="angle measured">—</span>`;
  const label = root.querySelector("label");
  const slider = root.querySelector("input");
  const [arm, ...words] = joint.name.split("_");
  const name = words.join(" ");
  label.textContent = name[0].toUpperCase() + name.slice(1);
  label.htmlFor = slider.id = joint.name;
  slider.setAttribute("aria-label", `${arm} ${name} target`);
  slider.min = Math.ceil(joint.min * DEG * 10) / 10;
  slider.max = Math.floor(joint.max * DEG * 10) / 10;
  slider.value = 0;
  root.querySelector("small").textContent = `${slider.min}° … ${slider.max}°`;
  for (const [event, names] of [["pointerenter", [joint.name]], ["pointerleave", []], ["focusin", [joint.name]], ["focusout", []]]) {
    root.addEventListener(event, () => viewer?.highlight("hover", names));
  }
  const row = { root, arm, slider, target: root.querySelector(".target"),
    measured: root.querySelector(".measured"), dragging: false, lastEdit: -Infinity };
  slider.addEventListener("pointerdown", () => { row.dragging = true; });
  for (const event of ["pointerup", "pointercancel", "blur"]) {
    slider.addEventListener(event, () => { row.dragging = false; });
  }
  slider.addEventListener("input", () => {
    row.lastEdit = performance.now();
    pending.set(joint.name, Number(slider.value) / DEG);
    row.target.textContent = `${Number(slider.value).toFixed(1)}°`;
    flushTargets();
  });
  row.index = index;
  rows.push(row);
  $("joints").append(root);
}

function playGesture(gesture) {
  if (!gesture) return;
  feedback("Checking gesture path…");
  send("gesture", { gesture });
}

function addGesture(gesture) {
  const group = gesture.id.startsWith("recorded-") ? $("recorded-gestures") : $("original-gestures");
  group.append(new Option(gesture.name, gesture.id));
  group.hidden = false;
  if (featured[gesture.id]) {
    const button = document.createElement("button");
    button.type = "button";
    button.className = "secondary";
    button.innerHTML = `<span aria-hidden="true">${featured[gesture.id]}</span>`;
    button.append(gesture.name);
    button.dataset.gesture = gesture.name;
    button.addEventListener("click", () => playGesture(gesture.id));
    $("featured-gestures").append(button);
  }
}

function resetRecorder() {
  recording = false;
  recorderPending = false;
  recordedPoses = 0;
  $("recorded-poses").textContent = "0 poses captured";
  $("gesture-recorder").reset();
}

function sendRecorder(action, values) {
  recorderPending = true;
  send(action, values);
  render();
}

function render() {
  const connection = state?.connection ?? "offline";
  const connected = connection === "connected";
  const mode = state?.mode ?? "arms";
  const switching = Boolean(state?.switching) || ["preparing", "settling", "supporting"].includes(mode);
  const fallen = mode === "fallen";
  const arms = mode === "arms" && !switching;
  const stopping = state?.motion === "stopping";
  const busy = ["planning", "gesture"].includes(state?.motion);
  const manual = arms && !readOnly && connected && state.owned && state.enabled && !busy;
  const canPlay = manual && state.motion === "idle";
  const owned = connected && state.owned;
  const label = { connected: "Connected", waiting: "Waiting for feedback", stale: "Stale feedback", offline: "Offline" }[connection];
  $("connection").textContent = [label, backendLabel].filter(Boolean).join(" · ");
  $("connection").dataset.tone = connected ? "good" : connection === "waiting" ? "neutral" : "danger";
  $("ownership").textContent = readOnly ? "Read only" :
    owned ? "You have control" : state?.control_taken ? "Another operator" : "Observation";
  const latched = state?.stop_latched;
  $("motion").textContent = fallen ? "Reset required" : stopping ? "Stopping" : latched ? "Stopped · Latched" :
    { manual: "Manual motion", planning: `Planning · ${state?.gesture}`,
      gesture: `Running · ${state?.gesture}`, driving: "Walking", idle: switching ? "Switching mode" : "Idle"
    }[state?.motion ?? "idle"];
  $("motion").dataset.tone = (latched || stopping || fallen) ? "danger" : busy || switching ? "busy" :
    manual || canDrive() ? "active" : "neutral";
  $("ownership").dataset.tone = owned ? "good" : "neutral";
  document.body.dataset.mode = !connected ? "offline" : latched || stopping || fallen ? "stopped" :
    busy || switching ? "gesture" : manual || canDrive() ? "manual" : "idle";
  $("controls-title").textContent = switching ? "Changing mode" : fallen ? "Locomotion stopped" :
    arms ? "Arm controls" : "Locomotion controls";
  $("lock").textContent = busy ? "Locked · gesture running" : arms ? "Degrees" : "";
  $("lock").dataset.tone = busy ? "busy" : "neutral";
  $("control-modes").hidden = !locomotion;
  for (const button of modeButtons) {
    const selected = (state?.switching === "reset" ? "arms" : state?.switching)
      ?? (["arms", "settling", "supporting"].includes(mode) ? "arms" : "locomotion");
    button.setAttribute("aria-pressed", String(button.dataset.controlMode === selected));
    button.disabled = !owned || latched || switching || stopping || fallen || button.dataset.controlMode === selected;
  }
  $("arm-controls").hidden = !arms;
  $("gestures-title").closest(".gestures").hidden = readOnly || !arms;
  $("drive-controls").hidden = arms || switching || fallen;
  $("drive-hint").textContent = canDrive() ? "Hold a button or key to move." :
    state?.enabled ? "Driving paused." : "Enable motion to drive.";
  for (const button of driveButtons) button.disabled = !canDrive();
  if (!canDrive()) clearDrive();
  const step = !owned ? 0 : state.enabled ? 2 : 1;
  [...$("steps").children].forEach((item, index) => {
    item.dataset.state = index < step ? "done" : index === step && connected ? "current" : "";
  });
  $("take-control").hidden = Boolean(owned);
  $("take-control").disabled = readOnly || !connected || state.control_taken || stopping;
  $("enable").hidden = !owned || state.enabled || latched || stopping || switching || fallen;
  $("reset-stop").hidden = !owned || !latched || stopping || switching || fallen;
  $("reset-sim").hidden = !owned || !fallen || switching;
  $("release-control").hidden = !owned;
  $("stop").disabled = readOnly || !owned || latched;
  $("ready").disabled = !canPlay;
  for (const button of $("featured-gestures").children) {
    button.disabled = !canPlay;
    button.toggleAttribute("data-active", busy && state.gesture === button.dataset.gesture);
  }
  $("gesture-select").disabled = !canPlay;
  $("play-gesture").disabled = !canPlay || !$("gesture-select").value;
  $("record-gesture").hidden = recording;
  $("record-gesture").disabled = !canPlay;
  $("gesture-recorder").hidden = !recording;
  $("capture-pose").disabled = !canPlay || recorderPending || Boolean(pending.size);
  $("save-recording").disabled = !arms || !owned || !recordedPoses || recorderPending;
  $("discard-recording").disabled = recorderPending || !connected;
  if (!manual) {
    pending.clear();
    for (const row of rows) row.dragging = false;
  }
  for (const row of rows) {
    row.slider.disabled = !manual;
    if (!arms) continue;
    row.root.hidden = row.arm !== side;
    const measured = state?.pose?.[state.pose.length - rows.length + row.index];
    const target = state?.target?.[row.index];
    const editing = manual && (row.dragging || performance.now() - row.lastEdit < 160);
    if (!editing) {
      row.target.textContent = angleText(target);
      row.slider.value = ((target ?? measured ?? 0) * DEG).toFixed(1);
    }
    row.measured.textContent = angleText(measured);
    const { min, max } = row.slider;
    row.root.toggleAttribute("data-live", Number.isFinite(measured));
    row.root.style.setProperty("--measured", Math.min(1, Math.max(0, (measured * DEG - min) / (max - min))));
  }
  if (viewer) {
    $("model-status").textContent = viewer.status ||
      (connected ? "Live feedback" : state?.pose ? "Last measured pose" : "Waiting for feedback");
  }
}

function connect() {
  socket = new WebSocket(`ws://${location.host}/control`);
  socket.addEventListener("open", () => { lastStateAt = performance.now(); });
  socket.addEventListener("message", ({ data }) => {
    const message = JSON.parse(data);
    if (message.type === "state") {
      lastStateAt = performance.now();
      const previous = state;
      state = message;
      // Browsers throttle timers in background tabs, so the lease is renewed from state packets instead.
      if (state.owned && lastStateAt - lastHeartbeat > 180) {
        lastHeartbeat = lastStateAt;
        send("heartbeat");
      }
      viewer?.showPose(state.pose, previous?.connection !== "connected" || state.connection !== "connected",
        state.target, state.base);
      if (state.fault && state.fault !== previous?.fault) feedback(state.fault, true);
      if (state.stop_latched && state.motion === "idle" && previous?.motion === "stopping") {
        feedback("Motion stopped. Reset Stop before enabling again.");
      }
      if (state.enabled && state.motion === "idle" && previous?.motion === "gesture") {
        feedback("Gesture complete. Arms returned to Ready.");
      }
      if (previous?.switching && !state.switching && state.owned && !state.fault && !state.stop_latched) {
        feedback(`${state.mode === "arms" ? "Arms" : "Locomotion"} mode ready. Enable motion when ready.`);
      }
      if (previous?.owned && !state.owned && !state.fault) feedback("Control released. Take control to move again.");
      render();
      return;
    }
    const [action, sentAt] = commands.get(message.id) ?? [];
    commands.delete(message.id);
    if (["capture_pose", "save_recording", "discard_recording"].includes(action)) {
      recorderPending = false;
      if (!message.ok) {
        feedback(message.message, true);
      } else if (action === "capture_pose") {
        recordedPoses = message.count;
        $("recorded-poses").textContent = `${recordedPoses} ${recordedPoses === 1 ? "pose" : "poses"} captured`;
      } else {
        resetRecorder();
        if (message.gesture) {
          addGesture(message.gesture);
          $("gesture-select").value = message.gesture.id;
          feedback(`Saved gesture “${message.gesture.name}”.`);
        }
      }
      render();
      return;
    }
    // A gesture is acknowledged only after its path is planned, so it is not a transport latency.
    if (action && action !== "gesture") $("latency").textContent = `${(performance.now() - sentAt).toFixed(1)} ms`;
    if (message.ok && action === "enable" && state?.mode === "locomotion") {
      feedback("Motion enabled. Hold a drive control to move.");
    } else if (message.ok && action === "stop" && state?.mode !== "arms") {
      feedback("Stop requested. Waiting for the robot to settle.");
    } else if (message.ok && messages[action]) {
      feedback(messages[action]);
    }
    if (!message.ok && action) {
      if (message.message === "Gesture planning was cancelled") return;
      if (action === "set_joint") rows.forEach(row => { row.lastEdit = -Infinity; });
      feedback(message.message, true);
    }
  });
  socket.addEventListener("close", () => {
    clearDrive();
    if (state?.pose) viewer?.showPose(state.pose, true, null, state.base);
    state = { pose: state?.pose, mode: state?.mode, base: state?.base, connection: "offline" };
    commands.clear();
    feedback(recording ? "Connection lost. The unsaved recording was cleared." :
      "Connection lost. Waiting to reconnect.", true);
    resetRecorder();
    render();
    setTimeout(connect, 900);
  });
}

$("gesture-select").addEventListener("change", render);
$("play-gesture").addEventListener("click", () => playGesture($("gesture-select").value));
$("record-gesture").addEventListener("click", () => {
  recording = true;
  feedback("Move the arms with the sliders, then capture each pose once the arms settle.");
  render();
});
$("capture-pose").addEventListener("click", () => {
  if (!pending.size) sendRecorder("capture_pose");
});
$("gesture-recorder").addEventListener("submit", event => {
  event.preventDefault();
  if (!$("save-recording").disabled) sendRecorder("save_recording", { name: $("gesture-name").value });
});
$("discard-recording").addEventListener("click", () => sendRecorder("discard_recording"));
$("ready").addEventListener("click", () => {
  feedback("Checking the path to Ready…");
  send("ready");
});
for (const button of document.querySelectorAll("[data-side]")) {
  button.addEventListener("click", () => {
    side = button.dataset.side;
    for (const tab of document.querySelectorAll("[data-side]")) {
      tab.setAttribute("aria-pressed", String(tab === button));
    }
    render();
  });
}
for (const button of modeButtons) {
  button.addEventListener("click", () => {
    clearDrive();
    pending.clear();
    send("set_mode", { mode: button.dataset.controlMode });
  });
}
$("reset-sim").addEventListener("click", () => send("set_mode", { mode: "reset" }));
for (const button of driveButtons) {
  button.addEventListener("pointerdown", event => {
    if (event.button !== 0 || !canDrive()) return;
    button.setPointerCapture(event.pointerId);
    held.set(event.pointerId, button.dataset.drive);
    sendDrive();
  });
  for (const name of ["pointerup", "pointercancel", "lostpointercapture"]) {
    button.addEventListener(name, event => releaseDrive(event.pointerId));
  }
}
for (const [id, command] of Object.entries({
  "take-control": "take_control", "release-control": "release_control", enable: "enable",
  stop: "stop", "reset-stop": "reset_stop",
})) {
  $(id).addEventListener("click", () => {
    if (command === "stop" || command === "release_control") {
      pending.clear();
      clearDrive();
    }
    send(command);
  });
}
addEventListener("keydown", event => {
  if (event.key === "Escape" && !$("stop").disabled) {
    $("stop").click();
    return;
  }
  if (!canDrive() || event.altKey || event.ctrlKey || event.metaKey ||
      event.target.closest?.("input, select, textarea, [contenteditable]")) return;
  const key = (event.code === "Space" || event.code === "Enter")
    ? event.target.dataset?.drive : event.code;
  if (!driveKeys[key]) return;
  event.preventDefault();
  if (!event.repeat) {
    held.set(event.code, key);
    sendDrive();
  }
});
addEventListener("keyup", event => releaseDrive(event.code));
addEventListener("blur", clearDrive);
addEventListener("pagehide", clearDrive);
document.addEventListener("visibilitychange", () => { if (document.hidden) clearDrive(); });
setInterval(() => { if (held.size && canDrive()) sendDrive(); }, 80);
setInterval(() => {
  if (socket?.readyState === WebSocket.OPEN && performance.now() - lastStateAt > 900) {
    socket.close();
  }
}, 300);

try {
  const config = await fetch("/config");
  if (!config.ok) throw new Error("Configuration unavailable");
  const { backend, read_only, locomotion: canSelectLocomotion, joints, gestures } = await config.json();
  readOnly = read_only;
  locomotion = canSelectLocomotion;
  $("backend").textContent = { sim: "MuJoCo", mock: "Mock", hardware: "Physical G1" }[backend];
  backendLabel = { sim: "SIM", mock: "MOCK", hardware: "HARDWARE" }[backend];
  if (readOnly) {
    document.body.classList.add("observation-only");
    $("motion").parentElement.hidden = true;
    $("latency").parentElement.hidden = true;
    $("steps").closest(".session").hidden = true;
    $("gestures-title").closest(".gestures").hidden = true;
    $("stop").hidden = true;
    document.querySelector(".view-hint").textContent = "Measured pose · Drag to orbit · Scroll to zoom";
    $("command-status").textContent = "Hardware observation is read-only.";
  }
  joints.forEach(addRow);
  for (const id of Object.keys(featured)) {
    const gesture = gestures.find(gesture => gesture.id === id);
    if (gesture) addGesture(gesture);
  }
  gestures.filter(gesture => !featured[gesture.id]).forEach(addGesture);
  render();
  connect();
} catch (error) {
  feedback(error.message, true);
}

try {
  viewer = await createViewer($("viewport"), render);
  viewer.showPose(state?.pose, true, state?.target, state?.base);
  $("reset-camera").disabled = false;
  $("reset-camera").addEventListener("click", viewer.resetCamera);
  render();
} catch (error) {
  $("model-status").textContent = `3D view unavailable · ${error.message}`;
}
