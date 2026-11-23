import { Vector3 } from "three";

function turn(a, b, c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

function hull(points) {
  points.sort((a, b) => a.x - b.x || a.y - b.y);
  function half(ordered) {
    const edge = [];
    for (const point of ordered) {
      while (edge.length >= 2 && turn(edge.at(-2), edge.at(-1), point) <= 0) edge.pop();
      edge.push(point);
    }
    return edge.slice(0, -1);
  }
  return half(points).concat(half(points.reverse()));
}

// Project a small sample of each link's real mesh using the same pose and camera as WebGL.
export function createPosePreview(canvas, robots) {
  const context = canvas.getContext("2d");
  if (!context) throw new Error("Canvas 2D unavailable");
  const samples = new Map();
  const links = [];
  for (const robot of robots) {
    robot.traverse(object => {
      if (!object.isMesh) return;
      const { geometry } = object;
      if (!samples.has(geometry)) {
        const vertices = geometry.getAttribute("position");
        const stride = Math.max(1, Math.floor(vertices.count / 160));
        const points = [];
        for (let i = 0; i < vertices.count; i += stride) {
          points.push(new Vector3().fromBufferAttribute(vertices, i));
        }
        samples.set(geometry, points);
      }
      links.push({ object, robot, points: samples.get(geometry) });
    });
  }

  const point = new Vector3();
  return (camera, focus) => {
    const { width, height } = canvas;
    context.fillStyle = "#0c1524";
    context.fillRect(0, 0, width, height);
    // A fixed world grid shows translation even while the camera follows the base.
    const gridX = Math.round(focus.x * 2);
    const gridZ = Math.round(focus.z * 2);
    context.fillStyle = "#34496a";
    for (let x = gridX - 10; x <= gridX + 10; x++) {
      for (let z = gridZ - 10; z <= gridZ + 10; z++) {
        point.set(x / 2, -0.012, z / 2).project(camera);
        if (Math.abs(point.z) <= 1 && Math.abs(point.x) <= 1 && Math.abs(point.y) <= 1) {
          context.fillRect((point.x + 1) * width / 2, (1 - point.y) * height / 2, 2, 2);
        }
      }
    }
    const outlines = [];
    for (const { object, robot, points } of links) {
      if (!robot.visible) continue;
      const projected = [];
      let depth = 0;
      for (const vertex of points) {
        point.copy(vertex).applyMatrix4(object.matrixWorld).project(camera);
        if (point.z < -1 || point.z > 1) continue;
        projected.push({ x: (point.x + 1) * width / 2, y: (1 - point.y) * height / 2 });
        depth += point.z;
      }
      if (projected.length >= 3) {
        outlines.push({ points: hull(projected), material: object.material, depth: depth / projected.length });
      }
    }
    outlines.sort((a, b) => b.depth - a.depth);
    for (const { points, material } of outlines) {
      context.beginPath();
      context.moveTo(points[0].x, points[0].y);
      for (const { x, y } of points.slice(1)) context.lineTo(x, y);
      context.closePath();
      context.globalAlpha = material.opacity;
      context.fillStyle = material.color.getStyle();
      context.fill();
      context.strokeStyle = material.transparent ? "#83b5ff" : "#8598af";
      context.stroke();
    }
    context.globalAlpha = 1;
  };
}
