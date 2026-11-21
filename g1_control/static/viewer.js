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
  scene.background = new THREE.Color(0x11243b);
  scene.fog = new THREE.Fog(0x11243b, 4, 11);
  const pivots = new Map();

  // Each hinge rotates around an anchor expressed in its owning body's frame.
  function makeBody(body) {
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
    for (const visual of body.visuals) {
      const material = new THREE.MeshStandardMaterial({
        color: new THREE.Color(...visual.color), roughness: 0.62, metalness: 0.22,
      });
      const mesh = new THREE.Mesh(geometry.get(visual.mesh), material);
      transform(mesh, visual.position, visual.quaternion);
      content.add(mesh);
    }
    for (const child of body.children) content.add(makeBody(child));
    return frame;
  }

  const robot = new THREE.Group();
  robot.rotation.x = -Math.PI / 2;
  robot.add(makeBody(root));
  scene.add(robot);
  if (joints.length !== 29 || joints.some(name => !pivots.has(name))) {
    throw new Error("G1 model does not match the DDS joint ordering");
  }
  const rig = joints.map(name => pivots.get(name));

  const grid = new THREE.GridHelper(12, 48, 0x4a627b, 0x334c67);
  grid.position.y = -0.012;
  scene.add(grid, new THREE.HemisphereLight(0xf0f6ff, 0x64758b, 2.2));
  const key = new THREE.DirectionalLight(0xffffff, 2.8);
  key.position.set(3, 6, 2);
  const rim = new THREE.DirectionalLight(0x91b9ff, 1.5);
  rim.position.set(-3, 3, -2);
  scene.add(key, rim);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.75));
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
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
    controls.update();
    renderer.render(scene, camera);
  });

  return {
    resetCamera: () => controls.reset(),
    showPose(angles, immediate = false) {
      if (!angles || angles.length !== joints.length) return;
      target = angles;
      if (immediate) displayed = target.slice();
    },
  };
}
