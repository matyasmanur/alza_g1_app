"""Extract a browser scene graph and visual meshes from Unitree's G1 MJCF."""

import json
import shutil
import sys
from pathlib import Path
from xml.etree import ElementTree


def numbers(element, attribute, default):
    """Read an optional MJCF vector."""
    return [float(value) for value in element.get(attribute, default).split()]


def export(source: Path, output: Path) -> None:
    """Preserve body transforms and hinge pivots without collision duplicates."""
    root = ElementTree.parse(source).getroot()
    meshes = {mesh.attrib["name"]: mesh.attrib["file"] for mesh in root.findall("./asset/mesh")}
    mesh_dir = source.parent / root.find("compiler").attrib["meshdir"]
    (output / "meshes").mkdir(parents=True, exist_ok=True)

    def collect(body):
        visuals = []
        for geom in body.findall("geom"):
            if geom.get("group") != "1" or geom.get("mesh") not in meshes:
                continue
            mesh = meshes[geom.attrib["mesh"]]
            shutil.copyfile(mesh_dir / mesh, output / "meshes" / mesh)
            visuals.append({
                "mesh": mesh,
                "position": numbers(geom, "pos", "0 0 0"),
                "quaternion": numbers(geom, "quat", "1 0 0 0"),
                "color": numbers(geom, "rgba", "0.64 0.66 0.72 1")[:3],
            })
        hinge = body.find("joint[@axis]")
        return {
            "name": body.attrib["name"],
            "position": numbers(body, "pos", "0 0 0"),
            "quaternion": numbers(body, "quat", "1 0 0 0"),
            "hinge": None if hinge is None else {
                "name": hinge.attrib["name"].removesuffix("_joint"),
                "position": numbers(hinge, "pos", "0 0 0"),
                "axis": numbers(hinge, "axis", "0 0 1"),
            },
            "visuals": visuals,
            "children": [collect(child) for child in body.findall("body")],
        }

    description = {
        "root": collect(root.find("./worldbody/body")),
        "joints": [motor.attrib["name"] for motor in root.findall("./actuator/motor")],
    }
    (output / "g1.json").write_text(json.dumps(description, separators=(",", ":")))


if __name__ == "__main__":
    export(Path(sys.argv[1]), Path(sys.argv[2]))
