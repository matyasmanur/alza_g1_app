import * as THREE from "three";
import { OrbitControls } from "/assets/vendor/OrbitControls.js";
import { STLLoader } from "/assets/vendor/STLLoader.js";
import { createPosePreview } from "/static/pose-preview.js?v=2";

function transform(object, position, [w, x, y, z]) {
  object.position.fromArray(position);
  object.quaternion.set(x, y, z, w).normalize();
}

function meshNames(body) {
  return [...body.visuals.map(({ mesh }) => mesh), ...body.children.flatMap(meshNames)];
}

export async function createViewer(container, onStatusChange) {
  const response = await fetch("/assets/g1.json");
  if (!response.ok) throw new Error("G1 model could not be loaded");
  const { root, joints } = await response.json();
  const loader = new STLLoader();
  const geometry = new Map(await Promise.all([...new Set(meshNames(root))].map(async name => [
    name, await loader.loadAsync(`/assets/meshes/${encodeURIComponent(name)}`),
  ])));
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0c1524);
  scene.fog = new THREE.Fog(0x0c1524, 4, 11);
  const links = new Map();
  const ghostMaterial = new THREE.MeshBasicMaterial({ color: 0x5b9dff, transparent: true, opacity: 0.32, depthWrite: false });

  // The ghost is a second copy of the arms only, posed at the commanded target.
  function makeRobot(ghost) {
    const pivots = new Map();

    // Each hinge rotates around an anchor expressed in its owning body's frame.
    function makeBody(body, arm) {
      arm ||= Boolean(body.hinge?.name.includes("shoulder_pitch"));
      const frame = new THREE.Group();
      transform(frame, body.position, body.quaternion);
      let content = frame;
      if (body.hinge) {
        const { name, position, axis } = body.hinge;
        const pivot = new THREE.Group();
        pivot.position.fromArray(position);
        content = new THREE.Group();
        content.position.copy(pivot.position).negate();
        frame.add(pivot);
        pivot.add(content);
        pivots.set(name, { pivot, axis: new THREE.Vector3(...axis).normalize() });
      }
      const materials = [];
      for (const visual of ghost && !arm ? [] : body.visuals) {
        const material = ghost ? ghostMaterial : new THREE.MeshStandardMaterial({
          color: new THREE.Color(...visual.color), roughness: 0.62, metalness: 0.22,
        });
        const mesh = new THREE.Mesh(geometry.get(visual.mesh), material);
        materials.push(material);
        transform(mesh, visual.position, visual.quaternion);
        content.add(mesh);
      }
      if (!ghost) links.set(body.name.replace(/_link$/, ""), materials);
      for (const child of body.children) content.add(makeBody(child, arm));
      return frame;
    }

    const robot = new THREE.Group();
    robot.rotation.x = -Math.PI / 2;
    const base = makeBody(root, false);
    robot.add(base);
    scene.add(robot);
    if (joints.length !== 29 || joints.some(name => !pivots.has(name))) {
      throw new Error("G1 model does not match the DDS joint ordering");
    }
    return { robot, base, rig: joints.map(name => pivots.get(name)) };
  }

  const { robot, base, rig } = makeRobot(false);
  const ghost = makeRobot(true);
  ghost.robot.visible = false;

  const grid = new THREE.GridHelper(48, 192, 0x34496a, 0x1d2c44);
  grid.position.y = -0.012;
  scene.add(grid, new THREE.HemisphereLight(0xf0f6ff, 0x64758b, 2.2));
  const key = new THREE.DirectionalLight(0xffffff, 2.8);
  key.position.set(3, 6, 2);
  const rim = new THREE.DirectionalLight(0x91b9ff, 1.5);
  rim.position.set(-3, 3, -2);
  scene.add(key, rim);

  let canvas = document.createElement("canvas");
  let renderer;
  let preview;
  let status = "";
  container.prepend(canvas);
  const camera = new THREE.PerspectiveCamera(34, 1, 0.03, 30);
  camera.position.set(2.12, 1.36, -1.65);
  const controls = new OrbitControls(camera, canvas);
  controls.target.set(0, 0.70, 0);
  controls.enableDamping = true;
  controls.minDistance = 0.75;
  controls.maxDistance = 6;
  controls.maxPolarAngle = Math.PI * 0.52;
  controls.update();
  controls.saveState();

  function usePosePreview() {
    renderer?.dispose();
    renderer = null;
    const replacement = document.createElement("canvas");
    replacement.width = canvas.width;
    replacement.height = canvas.height;
    controls.disconnect();
    canvas.replaceWith(replacement);
    canvas = replacement;
    controls.connect(canvas);
    preview = createPosePreview(canvas, [robot, ghost.robot]);
    status = "Simplified 3D view · WebGL unavailable";
    onStatusChange();
  }

  // Register before Three.js so it cannot start another restore cycle.
  canvas.addEventListener("webglcontextlost", event => {
    event.stopImmediatePropagation();
    usePosePreview();
  }, { once: true });
  try {
    const context = canvas.getContext("webgl2");
    if (context) renderer = new THREE.WebGLRenderer({ canvas, context });
  } catch (error) {
    console.warn("WebGL viewer unavailable", error);
  }
  if (renderer) {
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
  } else {
    usePosePreview();
  }

  new ResizeObserver(([entry]) => {
    const { width, height } = entry.contentRect;
    if (height === 0 || width === 0) return;
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    // Keep the drawing buffer bounded on Retina and large displays.
    const scale = Math.min(1, 1600 / Math.max(width, height));
    const bufferWidth = Math.round(width * scale);
    const bufferHeight = Math.round(height * scale);
    if (renderer) {
      renderer.setSize(bufferWidth, bufferHeight, false);
    } else {
      canvas.width = bufferWidth;
      canvas.height = bufferHeight;
    }
  }).observe(container);

  const glow = { hover: [], clash: [] };
  let goal = null;
  let target = Array(joints.length).fill(0);
  let displayed = target.slice();
  const basePosition = base.position.clone();
  const baseRotation = base.quaternion.clone();
  const travel = new THREE.Vector3();
  const delta = new THREE.Vector3();
  const renderedBase = base.position.clone();
  let instantBase = false;
  let lastFrame = performance.now();
  function draw(now) {
    requestAnimationFrame(draw);
    if (now - lastFrame < 1000 / 60 - 1) return;
    const step = 1 - Math.exp(-Math.max(0, now - lastFrame) / 32);
    lastFrame = now;
    rig.forEach(({ pivot, axis }, index) => {
      displayed[index] += (target[index] - displayed[index]) * step;
      pivot.quaternion.setFromAxisAngle(axis, displayed[index]);
    });
    base.position.lerp(basePosition, instantBase ? 1 : step);
    base.quaternion.slerp(baseRotation, instantBase ? 1 : step);
    instantBase = false;
    delta.subVectors(base.position, renderedBase).applyQuaternion(robot.quaternion);
    delta.y = 0;
    renderedBase.copy(base.position);
    travel.add(delta);
    camera.position.add(delta);
    controls.target.add(delta);
    // Arm joints are the last entries of the pose. Hide the ghost once the arms have arrived.
    const first = joints.length - (goal?.length ?? 0);
    ghost.robot.visible = Boolean(goal?.some((angle, index) => Math.abs(angle - displayed[first + index]) > 0.08));
    if (ghost.robot.visible) {
      ghost.base.position.copy(base.position);
      ghost.base.quaternion.copy(base.quaternion);
      ghost.rig.forEach(({ pivot, axis }, index) => {
        pivot.quaternion.setFromAxisAngle(axis, index < first ? displayed[index] : goal[index - first]);
      });
    }
    controls.update();
    if (renderer) {
      renderer.render(scene, camera);
    } else {
      scene.updateMatrixWorld();
      camera.updateMatrixWorld();
      preview(camera, controls.target);
    }
  }
  requestAnimationFrame(draw);

  return {
    get status() { return status; },
    resetCamera() {
      controls.reset();
      camera.position.add(travel);
      controls.target.add(travel);
      controls.update();
    },
    highlight(kind, names) {
      glow[kind] = names;
      for (const [name, materials] of links) {
        const lit = glow.hover.includes(name) || glow.clash.includes(name);
        for (const material of materials) {
          material.userData.color ??= material.color.getHex();
          material.color.setHex(lit ? 0xff2b2b : material.userData.color);
          material.emissive.setHex(lit ? 0xb00000 : 0);
        }
      }
    },
    showPose(angles, immediate = false, armTarget = null, worldBase = null) {
      goal = armTarget;
      if (worldBase) {
        const [w, x, y, z] = worldBase.quaternion;
        basePosition.fromArray(worldBase.position);
        baseRotation.set(x, y, z, w).normalize();
        instantBase ||= immediate;
      }
      if (!angles || angles.length !== joints.length) return;
      target = angles;
      if (immediate) displayed = target.slice();
    },
  };
}
