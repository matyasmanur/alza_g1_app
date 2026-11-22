import { createViewer } from "/static/viewer.js";

const $ = id => document.getElementById(id);
const DEG = 180 / Math.PI;
const rows = [];
const pending = new Map();
const commands = new Map();
const featured = { wave: "👋", "high-five": "🖐️", handshake: "🤝" };
const clock = () => new Date().toLocaleTimeString("en-GB");
let socket, state, viewer;
let modelError = false;
let nextId = 0;
let side = "left";
let backendLabel = "";
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
  stop: "Stop requested. Waiting for the arms to settle.",
  reset_stop: "Stop reset. Motion is disabled.",
};

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
  if (type !== "heartbeat") commands.set(id, [type, performance.now()]);
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

function addGestures(gestures) {
  const groups = ["Unitree-inspired", "Original"].map(label => {
    const group = document.createElement("optgroup");
    group.label = label;
    $("gesture-select").append(group);
    return group;
  });
  for (const gesture of gestures) {
    const option = new Option(gesture.name, gesture.id);
    groups[gesture.sdk_id === null ? 1 : 0].append(option);
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
}

function render() {
  const connection = state?.connection ?? "offline";
  const connected = connection === "connected";
  const stopping = state?.motion === "stopping";
  const busy = ["planning", "gesture"].includes(state?.motion);
  const manual = connected && state.owned && state.enabled && !busy;
  const canPlay = manual && state.motion === "idle";
  const owned = connected && state.owned;
  const label = { connected: "Connected", waiting: "Waiting for feedback", stale: "Stale feedback", offline: "Offline" }[connection];
  $("connection").textContent = [label, backendLabel].filter(Boolean).join(" · ");
  $("connection").dataset.tone = connected ? "good" : connection === "waiting" ? "neutral" : "danger";
  $("ownership").textContent = owned ? "You have control" : state?.control_taken ? "Another operator" : "Observation";
  const latched = state?.stop_latched;
  $("motion").textContent = stopping ? "Stopping" : latched ? "Stopped · Latched" :
    { manual: "Manual motion", planning: `Planning · ${state?.gesture}`,
      gesture: `Running · ${state?.gesture}`, idle: "Idle" }[state?.motion ?? "idle"];
  $("motion").dataset.tone = (latched || stopping) ? "danger" : busy ? "busy" : manual ? "active" : "neutral";
  $("ownership").dataset.tone = owned ? "good" : "neutral";
  document.body.dataset.mode = !connected ? "offline" : latched || stopping ? "stopped" :
    busy ? "gesture" : manual ? "manual" : "idle";
  $("lock").textContent = busy ? "Locked · gesture running" : "Degrees";
  $("lock").dataset.tone = busy ? "busy" : "neutral";
  const step = !owned ? 0 : state.enabled ? 2 : 1;
  [...$("steps").children].forEach((item, index) => {
    item.dataset.state = index < step ? "done" : index === step && connected ? "current" : "";
  });
  $("take-control").hidden = Boolean(owned);
  $("take-control").disabled = !connected || state.control_taken || stopping;
  $("enable").hidden = !owned || state.enabled || latched || stopping;
  $("reset-stop").hidden = !owned || !latched || stopping;
  $("release-control").hidden = !owned;
  $("stop").disabled = !owned || latched;
  $("ready").disabled = !canPlay;
  for (const button of $("featured-gestures").children) {
    button.disabled = !canPlay;
    button.toggleAttribute("data-active", busy && state.gesture === button.dataset.gesture);
  }
  $("gesture-select").disabled = !canPlay;
  $("play-gesture").disabled = !canPlay || !$("gesture-select").value;
  if (!manual) {
    pending.clear();
    for (const row of rows) row.dragging = false;
  }
  for (const row of rows) {
    row.root.hidden = row.arm !== side;
    row.slider.disabled = !manual;
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
  if (viewer && !modelError) {
    $("model-status").textContent = connected ? "Live feedback" : state?.pose ? "Last measured pose" : "Waiting for feedback";
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
      if (state.owned && lastStateAt - lastHeartbeat > 300) {
        lastHeartbeat = lastStateAt;
        send("heartbeat");
      }
      viewer?.showPose(state.pose, previous?.connection !== "connected" || state.connection !== "connected", state.target);
      if (state.fault && state.fault !== previous?.fault) feedback(state.fault, true);
      if (state.stop_latched && state.motion === "idle" && previous?.motion === "stopping") {
        feedback("Motion stopped. Reset Stop before enabling again.");
      }
      if (state.enabled && state.motion === "idle" && previous?.motion === "gesture") {
        feedback("Gesture complete. Arms returned to Ready.");
      }
      if (previous?.owned && !state.owned && !state.fault) feedback("Control released. Take control to move again.");
      render();
      return;
    }
    const [action, sentAt] = commands.get(message.id) ?? [];
    commands.delete(message.id);
    // A gesture is acknowledged only after its path is planned, so it is not a transport latency.
    if (action && action !== "gesture") $("latency").textContent = `${(performance.now() - sentAt).toFixed(1)} ms`;
    if (message.ok && messages[action]) feedback(messages[action]);
    if (!message.ok && action) {
      if (message.message === "Gesture planning was cancelled") return;
      if (action === "set_joint") rows.forEach(row => { row.lastEdit = -Infinity; });
      feedback(message.message, true);
    }
  });
  socket.addEventListener("close", () => {
    if (state?.pose) viewer?.showPose(state.pose, true);
    state = { pose: state?.pose, connection: "offline" };
    commands.clear();
    feedback("Connection lost. Waiting to reconnect.", true);
    render();
    setTimeout(connect, 900);
  });
}

$("gesture-select").addEventListener("change", render);
$("play-gesture").addEventListener("click", () => playGesture($("gesture-select").value));
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
for (const [id, command] of Object.entries({
  "take-control": "take_control", "release-control": "release_control", enable: "enable",
  stop: "stop", "reset-stop": "reset_stop",
})) {
  $(id).addEventListener("click", () => {
    if (command === "stop" || command === "release_control") pending.clear();
    send(command);
  });
}
addEventListener("keydown", event => {
  if (event.key === "Escape" && !$("stop").disabled) $("stop").click();
});
setInterval(() => {
  if (socket?.readyState === WebSocket.OPEN && performance.now() - lastStateAt > 900) {
    socket.close();
  }
}, 300);

try {
  const config = await fetch("/config");
  if (!config.ok) throw new Error("Configuration unavailable");
  const { backend, joints, gestures } = await config.json();
  $("backend").textContent = backend === "sim" ? "MuJoCo" : "Mock";
  backendLabel = backend === "sim" ? "SIM" : "MOCK";
  joints.forEach(addRow);
  addGestures(gestures);
  render();
  connect();
} catch (error) {
  feedback(error.message, true);
}

try {
  viewer = await createViewer($("viewport"));
  viewer.showPose(state?.pose, true);
  $("reset-camera").disabled = false;
  $("reset-camera").addEventListener("click", viewer.resetCamera);
  render();
} catch (error) {
  modelError = true;
  $("model-status").textContent = `3D view unavailable · ${error.message}`;
}
