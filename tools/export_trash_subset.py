#!/usr/bin/env python3
"""Export a small TrashKit subset as Gazebo models.

Run with Blender from the repository root:
  blender -b /mnt/t/Trashkit/Trashkit/TrashKit.blend --python tools/export_trash_subset.py
"""

import os
import shutil
import xml.sax.saxutils as xml_utils

import bpy


MODELS = [
    {
        "model_name": "trash_bottle_01",
        "object_name": "Bottle_A_C_Lying",
        "label": "TrashKit plastic bottle",
        "scale": 1.0,
    },
    {
        "model_name": "trash_box_01",
        "object_name": "Box_Paper_Closed_A",
        "label": "TrashKit paper box",
        "scale": 0.55,
    },
    {
        "model_name": "trash_can_01",
        "object_name": "Can_Beverage_Damaged_01.005",
        "label": "TrashKit beverage can",
        "scale": 1.0,
    },
]

ENABLE_COLLISION = False


def repo_root():
    return os.getcwd()


def select_only(obj):
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj


def export_collada(obj, filepath):
    select_only(obj)
    bpy.ops.wm.collada_export(
        filepath=filepath,
        selected=True,
        apply_modifiers=True,
        include_children=False,
        include_armatures=False,
        include_shapekeys=False,
        use_object_instantiation=False,
    )


def model_config(model_name, label):
    escaped_name = xml_utils.escape(model_name)
    escaped_label = xml_utils.escape(label)
    return f"""<?xml version="1.0"?>
<model>
  <name>{escaped_name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>Gen0</name>
  </author>
  <description>{escaped_label}</description>
</model>
"""


def model_sdf(model_name, mesh_file, dims, scale):
    sx, sy, sz = (max(0.01, value * scale) for value in dims)
    z_pose = sz / 2.0
    escaped_name = xml_utils.escape(model_name)
    escaped_mesh = xml_utils.escape(mesh_file)
    collision = ""
    if ENABLE_COLLISION:
        collision = f"""
      <collision name="collision">
        <geometry>
          <box>
            <size>{sx:.4f} {sy:.4f} {sz:.4f}</size>
          </box>
        </geometry>
      </collision>"""
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{escaped_name}">
    <static>true</static>
    <pose>0 0 {z_pose:.4f} 0 0 0</pose>
    <link name="link">{collision}
      <visual name="visual">
        <cast_shadows>false</cast_shadows>
        <geometry>
          <mesh>
            <uri>model://{escaped_name}/meshes/{escaped_mesh}</uri>
            <scale>{scale:.4f} {scale:.4f} {scale:.4f}</scale>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
"""


def write_text(path, contents):
    with open(path, "w", encoding="utf-8") as output:
        output.write(contents)


def main():
    output_root = os.path.join(
        repo_root(), "gen0_gz_sim_ros2", "gen0_main", "models"
    )
    os.makedirs(output_root, exist_ok=True)

    for item in MODELS:
        obj = bpy.data.objects.get(item["object_name"])
        if obj is None:
            raise RuntimeError(f"Missing object: {item['object_name']}")
        if obj.type != "MESH":
            raise RuntimeError(f"Object is not a mesh: {item['object_name']}")

        model_name = item["model_name"]
        model_dir = os.path.join(output_root, model_name)
        mesh_dir = os.path.join(model_dir, "meshes")
        if os.path.exists(model_dir):
            shutil.rmtree(model_dir)
        os.makedirs(mesh_dir, exist_ok=True)

        mesh_file = f"{model_name}.dae"
        mesh_path = os.path.join(mesh_dir, mesh_file)
        export_collada(obj, mesh_path)

        dims = tuple(float(value) for value in obj.dimensions)
        scale = float(item["scale"])
        write_text(os.path.join(model_dir, "model.config"), model_config(model_name, item["label"]))
        write_text(os.path.join(model_dir, "model.sdf"), model_sdf(model_name, mesh_file, dims, scale))

        print(
            f"Exported {item['object_name']} -> {model_name}, "
            f"dims={tuple(round(v, 4) for v in dims)}, scale={scale}"
        )


if __name__ == "__main__":
    main()
