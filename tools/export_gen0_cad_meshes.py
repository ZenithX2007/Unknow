"""Convert the SolidWorks GLB exports into Gazebo-ready COLLADA meshes.

Run with Blender, not the system Python:
  blender --background --factory-startup --python tools/export_gen0_cad_meshes.py
"""

from pathlib import Path

import bpy
from mathutils import Matrix


ROOT = Path(__file__).resolve().parents[1]
MESH_DIR = ROOT / "gen0_gz_sim_ros2/gen0_main/meshes/gen0_cad"
CAR_SOURCE = MESH_DIR / "car_model_source.glb"
TYRE_SOURCE = MESH_DIR / "tyre_source.glb"
CAR_OUTPUT = MESH_DIR / "gen0_body.dae"
TYRE_OUTPUT = MESH_DIR / "gen0_tyre.dae"
BODY_TEXTURE = MESH_DIR / "body_black_softtouch.png"
HUB_TEXTURE = MESH_DIR / "hub_light_machined.png"

# In the CAD assembly, -X is forward, Z is left, and +Y points downward. The
# origin below is the midpoint between the four tyre hubs, on the ground.
CAD_TO_GAZEBO = Matrix(
    (
        (-1.0, 0.0, 0.0, -0.161248),
        (0.0, 0.0, 1.0, -0.093959),
        (0.0, -1.0, 0.0, 0.333000),
        (0.0, 0.0, 0.0, 1.0),
    )
)
EXCLUDED_BODY_NAMES = ("cube", "camera", "mid-360", "tyre")
BODY_DECIMATE_RATIO = 0.35
BODY_SHELL_NAMES = ("下面盖子-1", "底盘-1", "垃圾箱-1")
HUB_NAME_PREFIX = "wheel-"
TOP_SHELL_NAME = "上面壳-1"


def reset_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)


def import_glb(path):
    bpy.ops.import_scene.gltf(filepath=str(path))
    bpy.context.view_layer.update()


def mesh_objects():
    return [obj for obj in bpy.context.scene.objects if obj.type == "MESH"]


def bake_world_transform(obj):
    obj.data.transform(obj.matrix_world)
    obj.matrix_world = Matrix.Identity(4)


def decimate(obj, ratio):
    modifier = obj.modifiers.new(name="gazebo_mesh_decimate", type="DECIMATE")
    modifier.ratio = ratio
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.modifier_apply(modifier=modifier.name)


def configure_rubber_materials():
    for material in bpy.data.materials:
        material.use_nodes = False
        material.diffuse_color = (0.018, 0.022, 0.026, 1.0)
        material.specular_color = (0.12, 0.12, 0.12)
        material.specular_intensity = 0.32
        material.roughness = 0.58


def textured_material(name, texture_path, roughness, metallic):
    image = bpy.data.images.load(str(texture_path), check_existing=True)
    material = bpy.data.materials.new(name=name)
    material.use_nodes = True
    material.blend_method = "OPAQUE"
    nodes = material.node_tree.nodes
    links = material.node_tree.links
    for node in nodes:
        nodes.remove(node)
    output = nodes.new("ShaderNodeOutputMaterial")
    shader = nodes.new("ShaderNodeBsdfPrincipled")
    texture = nodes.new("ShaderNodeTexImage")
    texture.image = image
    texture.extension = "REPEAT"
    shader.inputs["Roughness"].default_value = roughness
    shader.inputs["Metallic"].default_value = metallic
    links.new(texture.outputs["Color"], shader.inputs["Base Color"])
    links.new(shader.outputs["BSDF"], output.inputs["Surface"])
    return material


def assign_material(obj, material):
    if not obj.data.materials:
        obj.data.materials.append(material)
        return
    for index in range(len(obj.data.materials)):
        obj.data.materials[index] = material


def dark_detail_material():
    material = bpy.data.materials.new(name="gen0_dark_detail")
    material.use_nodes = False
    material.diffuse_color = (0.018, 0.021, 0.024, 1.0)
    material.specular_color = (0.0, 0.0, 0.0)
    material.specular_intensity = 0.0
    material.roughness = 1.0
    return material


def darken_white_edge_materials(obj, material):
    for index, source in enumerate(obj.data.materials):
        if source and source.name.lower().startswith("whitehighglossplastic"):
            obj.data.materials[index] = material


def apply_top_shell_materials(obj, body_material):
    """Keep only the CAD roof's white faces; make its remaining shell matte black."""
    for index, source in enumerate(obj.data.materials):
        if not source or source.name != "whitehighglossplastic.008":
            obj.data.materials[index] = body_material


def export_collada(path):
    bpy.ops.wm.collada_export(
        filepath=str(path),
        selected=False,
        apply_modifiers=True,
        include_children=False,
    )


def export_body():
    reset_scene()
    import_glb(CAR_SOURCE)
    objects = [
        obj
        for obj in mesh_objects()
        if not any(token in obj.name.lower() for token in EXCLUDED_BODY_NAMES)
    ]
    body_material = textured_material(
        "gen0_body_black_softtouch", BODY_TEXTURE, roughness=0.82, metallic=0.0
    )
    hub_material = textured_material(
        "gen0_hub_light_machined", HUB_TEXTURE, roughness=0.34, metallic=0.48
    )
    edge_material = dark_detail_material()
    for obj in objects:
        if obj.name == TOP_SHELL_NAME:
            apply_top_shell_materials(obj, body_material)
        elif obj.name in BODY_SHELL_NAMES:
            assign_material(obj, body_material)
        elif obj.name.lower().startswith(HUB_NAME_PREFIX):
            assign_material(obj, hub_material)
        elif obj.name != TOP_SHELL_NAME:
            darken_white_edge_materials(obj, edge_material)
        bake_world_transform(obj)
        obj.data.transform(CAD_TO_GAZEBO)
        # CAD-to-Gazebo changes handedness; preserve outward-facing surfaces.
        obj.data.flip_normals()
        decimate(obj, BODY_DECIMATE_RATIO)
    for obj in mesh_objects():
        if obj not in objects:
            mesh = obj.data
            bpy.data.objects.remove(obj, do_unlink=True)
            if mesh.users == 0:
                bpy.data.meshes.remove(mesh)
    export_collada(CAR_OUTPUT)


def export_tyre():
    reset_scene()
    import_glb(TYRE_SOURCE)
    objects = mesh_objects()
    for obj in objects:
        bake_world_transform(obj)
    configure_rubber_materials()
    export_collada(TYRE_OUTPUT)


export_body()
export_tyre()
