import * as THREE from "three";
import { OrbitControls } from "/assets/vendor/OrbitControls.js";
import { STLLoader } from "/assets/vendor/STLLoader.js";

function transform(object, position, [w, x, y, z]) {
  object.position.fromArray(position);
  object.quaternion.set(x, y, z, w).normalize();
}

function meshNames(body) {
  return [...body.visuals.map(({ mesh }) => mesh), ...body.children.flatMap(meshNames)];
}

export async function createViewer(container) {
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
        mesh.castShadow = !ghost;
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
    robot.add(makeBody(root, false));
    scene.add(robot);
    if (joints.length !== 29 || joints.some(name => !pivots.has(name))) {
      throw new Error("G1 model does not match the DDS joint ordering");
    }
    return { robot, rig: joints.map(name => pivots.get(name)) };
  }

  const { rig } = makeRobot(false);
  const ghost = makeRobot(true);
  ghost.robot.visible = false;

  const grid = new THREE.GridHelper(12, 48, 0x34496a, 0x1d2c44);
  grid.position.y = -0.012;
  scene.add(grid, new THREE.HemisphereLight(0xf0f6ff, 0x64758b, 2.2));
  const key = new THREE.DirectionalLight(0xffffff, 2.8);
  key.position.set(3, 6, 2);
  key.castShadow = true;
  key.shadow.mapSize.set(1024, 1024);
  Object.assign(key.shadow.camera, { left: -2, right: 2, top: 2, bottom: -2 });
  const floor = new THREE.Mesh(new THREE.CircleGeometry(4, 48), new THREE.ShadowMaterial({ opacity: 0.45 }));
  floor.rotation.x = -Math.PI / 2;
  floor.position.y = -0.01;
  floor.receiveShadow = true;
  const rim = new THREE.DirectionalLight(0x91b9ff, 1.5);
  rim.position.set(-3, 3, -2);
  scene.add(key, rim, floor);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.75));
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.shadowMap.enabled = true;
  container.prepend(renderer.domElement);
  const camera = new THREE.PerspectiveCamera(34, 1, 0.03, 30);
  camera.position.set(2.12, 1.36, -1.65);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0.70, 0);
  controls.enableDamping = true;
  controls.minDistance = 0.75;
  controls.maxDistance = 6;
  controls.maxPolarAngle = Math.PI * 0.52;
  controls.update();
  controls.saveState();
  new ResizeObserver(([entry]) => {
    const { width, height } = entry.contentRect;
    if (height === 0 || width === 0) return;
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height, false);
  }).observe(container);

  const glow = { hover: [], clash: [] };
  let goal = null;
  let target = Array(joints.length).fill(0);
  let displayed = target.slice();
  let lastFrame = performance.now();
  renderer.setAnimationLoop(now => {
    const step = 1 - Math.exp(-Math.max(0, now - lastFrame) / 32);
    lastFrame = now;
    rig.forEach(({ pivot, axis }, index) => {
      displayed[index] += (target[index] - displayed[index]) * step;
      pivot.quaternion.setFromAxisAngle(axis, displayed[index]);
    });
    // Arm joints are the last entries of the pose. Hide the ghost once the arms have arrived.
    const first = joints.length - (goal?.length ?? 0);
    ghost.robot.visible = Boolean(goal?.some((angle, index) => Math.abs(angle - displayed[first + index]) > 0.08));
    if (ghost.robot.visible) {
      ghost.rig.forEach(({ pivot, axis }, index) => {
        pivot.quaternion.setFromAxisAngle(axis, index < first ? displayed[index] : goal[index - first]);
      });
    }
    controls.update();
    renderer.render(scene, camera);
  });

  return {
    resetCamera: () => controls.reset(),
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
    showPose(angles, immediate = false, armTarget = null) {
      goal = armTarget;
      if (!angles || angles.length !== joints.length) return;
      target = angles;
      if (immediate) displayed = target.slice();
    },
  };
}
