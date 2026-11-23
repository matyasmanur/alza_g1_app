import assert from "node:assert/strict";
import { execFileSync } from "node:child_process";
import { writeFile } from "node:fs/promises";
import { chromium } from "playwright";

const mode = process.argv[2];
assert(["benchmark", "demo", "locomotion"].includes(mode), "Choose benchmark, demo, or locomotion");
const url = "http://127.0.0.1:8000";
let serverReady = false;
for (let attempt = 0; attempt < 40 && !serverReady; attempt++) {
  const response = await fetch(`${url}/config`).catch(() => null);
  serverReady = response?.ok ?? false;
  if (!serverReady) await new Promise(resolve => setTimeout(resolve, 250));
}
assert(serverReady, "MuJoCo console is not listening in the shared network namespace");

const browser = await chromium.launch({ args: [
  "--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader", "--no-sandbox",
] });
const context = await browser.newContext({
  viewport: { width: 1280, height: 940 },
  ...(mode === "demo" ? { recordVideo: { dir: "/tmp/g1-demo", size: { width: 1280, height: 940 } } } : {}),
});
const page = await context.newPage();
const pageErrors = [];
page.on("pageerror", error => pageErrors.push(error.message));

function text(selector, expected) {
  return page.waitForFunction(({ selector, expected }) => document.querySelector(selector)?.textContent === expected,
    { selector, expected }, { timeout: 35000 });
}

async function clickAngle(joint, degrees) {
  const slider = page.locator(`#${joint}`);
  const [min, max] = await slider.evaluate(input => [Number(input.min), Number(input.max)]);
  const box = await slider.boundingBox();
  const fraction = (degrees - min) / (max - min);
  // Leave the range input's half-thumb width at each end of its track.
  await slider.click({ position: { x: 8 + fraction * (box.width - 16), y: box.height / 2 } });
}

async function observeMotion(joint, below) {
  await page.waitForFunction(({ joint, below }) => {
    const row = document.getElementById(joint).closest(".joint");
    return parseFloat(row.querySelector(".measured").textContent) < below;
  }, { joint, below });
}

async function benchmark() {
  const samples = [];
  for (let index = 0; index < 35; index++) {
    await text("#motion", "Idle");
    await page.waitForTimeout(140 + (index * 17) % 31);
    await clickAngle("left_elbow", index % 2 ? 6 : 18);
    const ready = await page.waitForFunction(count => window.g1Timings.length > count, index,
      { timeout: 5000 });
    await ready.dispose();
    const sample = await page.evaluate(index => window.g1Timings[index], index);
    assert(sample.click_to_ack_ms >= 0 && sample.receive_to_write_ms >= 0);
    assert(sample.receive_to_motion_ms > sample.receive_to_write_ms);
    samples.push(sample);
  }
  const measured = samples.slice(5);
  const columns = ["click_to_ack_ms", "receive_to_write_ms", "receive_to_motion_ms"];
  const rows = measured.map((sample, index) => [index + 1, sample.id, sample.target_deg.toFixed(3),
    ...columns.map(key => sample[key].toFixed(3))].join(","));
  await writeFile("/out/latency.csv", [`sample,command_id,target_deg,${columns.join(",")}`, ...rows, ""].join("\n"));
  for (const key of columns) {
    const sorted = measured.map(sample => sample[key]).sort((a, b) => a - b);
    const median = (sorted[14] + sorted[15]) / 2;
    console.log(`${key} N=${sorted.length}, median=${median.toFixed(1)}, p95=${sorted[Math.ceil(0.95 * sorted.length) - 1].toFixed(1)}`);
  }
}

async function demo() {
  await clickAngle("left_shoulder_roll", -50);
  await page.waitForFunction(() => document.querySelector("#command-status").textContent.startsWith("Insufficient clearance"));
  await page.waitForTimeout(1100);
  await clickAngle("left_elbow", 18);
  await text("#motion", "Manual motion");
  await text("#motion", "Idle");
  await page.getByRole("button", { name: "Face wave" }).click();
  await text("#motion", "Running · Face wave");
  assert(await page.locator("#left_elbow").isDisabled());
  assert(await page.getByRole("button", { name: "High five" }).isDisabled());
  await observeMotion("left_shoulder_pitch", -26);
  await page.screenshot({ path: "/out/demo.png" });
  await text("#command-status", "Gesture complete. Arms returned to Ready.");
  await page.waitForTimeout(850);
  await page.locator("#gesture-select").selectOption("recorded-double-salute");
  await page.getByRole("button", { name: "Play gesture" }).click();
  await text("#motion", "Running · Double salute");
  await observeMotion("left_shoulder_pitch", -13);
  await page.getByRole("button", { name: "Stop motion" }).click();
  await text("#motion", "Stopped · Latched");
  assert(await page.locator("#left_elbow").isDisabled());
  await page.waitForTimeout(1600);
}

async function locomotion() {
  const last = () => page.evaluate(() => window.g1State);
  const isMode = expected => page.waitForFunction(expected => {
    const state = window.g1State;
    return state?.mode === expected && !state.switching;
  }, expected);
  const enable = page.getByRole("button", { name: "Enable motion" });
  const elbow = page.locator("#left_elbow");
  await clickAngle("left_elbow", 18);
  await text("#motion", "Manual motion");
  await page.getByRole("button", { name: "Locomotion", exact: true }).click();
  await isMode("locomotion");
  assert(await elbow.isDisabled());
  assert(await page.getByRole("button", { name: "Face wave", includeHidden: true }).isHidden());
  const forward = page.locator('[data-drive="KeyW"]');
  assert(await forward.isDisabled());
  await enable.click();
  await forward.hover();
  const initial = await last();
  await page.mouse.down();
  await text("#motion", "Walking");
  await page.waitForFunction(x => window.g1State.base.position[0] > x + 0.35, initial.base.position[0]);
  await page.mouse.up();
  await text("#motion", "Idle");

  const beforeTurn = (await last()).base.quaternion;
  await page.keyboard.down("q");
  await page.waitForFunction(q => Math.abs(window.g1State.base.quaternion.reduce(
    (sum, value, i) => sum + value * q[i], 0)) < 0.985, beforeTurn);
  await page.keyboard.press("Escape");
  await page.keyboard.up("q");
  await text("#motion", "Stopped · Latched");
  assert(await forward.isDisabled());
  assert(await elbow.isDisabled());
  await page.getByRole("button", { name: "Reset Stop" }).click();
  await enable.click();
  await page.waitForFunction(() => !document.querySelector('[data-drive="KeyW"]').disabled);

  const beforeSide = (await last()).base;
  await page.keyboard.down("d");
  await page.waitForFunction(({ position, quaternion }) => {
    const [w, x, y, z] = quaternion;
    const heading = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    const [nextX, nextY] = window.g1State.base.position;
    return Math.sin(heading) * (nextX - position[0]) - Math.cos(heading) * (nextY - position[1]) > 0.14;
  }, beforeSide);
  await page.keyboard.up("d");
  await text("#motion", "Idle");

  const beforeBlur = (await last()).base.position;
  await page.keyboard.down("w");
  await text("#motion", "Walking");
  await page.waitForFunction(([x, y]) => {
    const [nextX, nextY] = window.g1State.base.position;
    return Math.hypot(nextX - x, nextY - y) > 0.12;
  }, beforeBlur);
  await page.evaluate(() => dispatchEvent(new Event("blur")));
  await text("#motion", "Idle");
  await page.keyboard.up("w");
  await page.waitForTimeout(1300);
  await page.waitForFunction(() => Math.hypot(...window.g1State.base.velocity.slice(0, 2)) < 0.08,
    undefined, { timeout: 4000 });

  await page.getByRole("button", { name: "Arms", exact: true }).click();
  await isMode("arms");
  const landed = await last();
  assert(landed.base.position[0] - initial.base.position[0] > 0.35);
  assert(Math.abs(landed.base.quaternion[3]) > 0.12);
  await enable.click();
  await page.getByRole("button", { name: "Face wave" }).click();
  await text("#motion", "Running · Face wave");
  await observeMotion("left_shoulder_pitch", -26);
  await text("#command-status", "Gesture complete. Arms returned to Ready.");
  console.log("Verified arm motion, mode handoffs, walking, strafing, turning, Stop, blur, and a gesture at the new position");
}

if (mode === "benchmark") {
  await page.addInitScript(() => {
    window.g1Timings = [];
    const pending = new Map();
    const attached = new WeakSet();
    let clickedAt;
    document.addEventListener("pointerdown", event => {
      if (event.target.id === "left_elbow") clickedAt = event.timeStamp;
    }, true);
    const send = WebSocket.prototype.send;
    WebSocket.prototype.send = function (data) {
      const command = JSON.parse(data);
      if (command.type === "set_joint" && command.joint === "left_elbow") {
        pending.set(command.id, { clickedAt, target_deg: command.angle * 180 / Math.PI });
      }
      if (!attached.has(this)) {
        attached.add(this);
        this.addEventListener("message", event => {
          const packet = JSON.parse(event.data);
          const sample = pending.get(packet.id);
          if (!sample) return;
          if (packet.type === "ack" && packet.ok) sample.click_to_ack_ms = performance.now() - sample.clickedAt;
          if (packet.type === "timing") {
            pending.delete(packet.id);
            window.g1Timings.push({ ...sample, ...packet });
          }
        });
      }
      return send.call(this, data);
    };
  });
}

if (mode === "locomotion") {
  await page.addInitScript(() => {
    const Socket = window.WebSocket;
    window.g1Events = [];
    window.WebSocket = class extends Socket {
      constructor(...args) {
        super(...args);
        this.addEventListener("close", event => window.g1Events.push([Math.round(performance.now()), "close", event.code]));
        this.addEventListener("message", ({ data }) => {
          const packet = JSON.parse(data);
          if (packet.type === "state") {
            const change = [packet.mode, packet.switching, packet.motion, packet.owned, packet.fault].join("|");
            if (change !== this.change) window.g1Events.push([Math.round(performance.now()), change]);
            this.change = change;
            window.g1State = packet;
          } else if (packet.type === "ack" && !packet.ok) {
            window.g1Events.push([Math.round(performance.now()), packet]);
          }
        });
      }

      send(data) {
        const { type } = JSON.parse(data);
        if (type === "heartbeat" && performance.now() - (this.lastBeat ?? 0) > 450) {
          window.g1Events.push([Math.round(performance.now()), "heartbeat gap", Math.round(performance.now() - (this.lastBeat ?? 0))]);
        }
        if (type === "heartbeat") this.lastBeat = performance.now();
        super.send(data);
      }
    };
  });
}

try {
  await page.goto(url, { waitUntil: "domcontentloaded" });
  await text("#backend", "MuJoCo");
  await text("#model-status", "Live feedback");
  await page.getByRole("button", { name: "Take control" }).click();
  await page.getByRole("button", { name: "Enable motion" }).click();
  await page.waitForFunction(() => !document.querySelector("#left_elbow").disabled);
  if (mode === "benchmark") {
    await benchmark();
  } else if (mode === "demo") {
    await demo();
  } else {
    await locomotion();
  }
  assert.deepEqual(pageErrors, []);
} catch (error) {
  if (mode === "locomotion") {
    console.error(await page.evaluate(() => ({
      state: window.g1State,
      activity: document.getElementById("command-status").textContent,
      events: window.g1Events,
    })), pageErrors);
  }
  throw error;
} finally {
  await context.close();
  await browser.close();
}

if (mode === "demo") {
  execFileSync("ffmpeg", ["-y", "-loglevel", "error", "-i", await page.video().path(),
    "-c:v", "libx264", "-preset", "fast", "-crf", "22", "-pix_fmt", "yuv420p",
    "-movflags", "+faststart", "-an", "/out/demo.mp4"], { stdio: "inherit" });
  console.log("Saved MuJoCo demo recording and screenshot");
}
