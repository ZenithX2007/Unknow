"""Read the visual center of exported Gazebo trash models."""

from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET


COLLADA_NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


@dataclass(frozen=True)
class TrashModelGeometry:
    """Visual center offset from the Gazebo model frame."""

    center_offset_x: float
    center_offset_y: float


def load_trash_model_geometry(package_share, model_name):
    """Return the visual center offset for one installed Gazebo model."""
    model_dir = Path(package_share) / "models" / model_name
    sdf_path = model_dir / "model.sdf"
    if not sdf_path.exists():
        raise FileNotFoundError(sdf_path)

    sdf_root = ET.parse(sdf_path).getroot()
    model_element = sdf_root.find("model")
    if model_element is None:
        raise ValueError(f"Missing model element in {sdf_path}")

    model_pose = _parse_vector(model_element.findtext("pose"), 6, [0.0] * 6)
    mesh_element = model_element.find(".//mesh")
    if mesh_element is None:
        return TrashModelGeometry(model_pose[0], model_pose[1])

    scale = _parse_vector(mesh_element.findtext("scale"), 3, [1.0, 1.0, 1.0])
    mesh_uri = (mesh_element.findtext("uri") or "").strip()
    mesh_name = mesh_uri.rsplit("/", 1)[-1]
    if not mesh_name:
        raise ValueError(f"Missing mesh URI in {sdf_path}")

    mesh_path = model_dir / "meshes" / mesh_name
    center = _collada_visual_center(mesh_path)
    return TrashModelGeometry(
        model_pose[0] + center[0] * scale[0],
        model_pose[1] + center[1] * scale[1],
    )


def _parse_vector(text, length, default):
    if not text:
        return list(default)
    values = [float(value) for value in text.split()]
    if len(values) == 1:
        values *= length
    if len(values) < length:
        raise ValueError(f"Expected {length} values, got {values}")
    return values[:length]


def _identity_matrix():
    return (
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
    )


def _matrix_multiply(left, right):
    return tuple(
        sum(left[row * 4 + index] * right[index * 4 + column] for index in range(4))
        for row in range(4)
        for column in range(4)
    )


def _matrix_transform(matrix, point):
    x, y, z = point
    return (
        matrix[0] * x + matrix[1] * y + matrix[2] * z + matrix[3],
        matrix[4] * x + matrix[5] * y + matrix[6] * z + matrix[7],
        matrix[8] * x + matrix[9] * y + matrix[10] * z + matrix[11],
    )


def _local_transform(element):
    transform = _identity_matrix()
    for child in list(element):
        tag = child.tag.rsplit("}", 1)[-1]
        if tag == "matrix":
            values = [float(value) for value in (child.text or "").split()]
            if len(values) != 16:
                raise ValueError("Invalid Collada matrix")
            local = tuple(values)
        elif tag == "translate":
            x, y, z = _parse_vector(child.text, 3, [0.0, 0.0, 0.0])
            local = (
                1.0,
                0.0,
                0.0,
                x,
                0.0,
                1.0,
                0.0,
                y,
                0.0,
                0.0,
                1.0,
                z,
                0.0,
                0.0,
                0.0,
                1.0,
            )
        elif tag == "scale":
            x, y, z = _parse_vector(child.text, 3, [1.0, 1.0, 1.0])
            local = (
                x,
                0.0,
                0.0,
                0.0,
                0.0,
                y,
                0.0,
                0.0,
                0.0,
                0.0,
                z,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
            )
        elif tag == "rotate":
            values = _parse_vector(child.text, 4, [0.0, 0.0, 1.0, 0.0])
            local = _rotation_matrix(values[0], values[1], values[2], values[3])
        else:
            continue
        transform = _matrix_multiply(transform, local)
    return transform


def _rotation_matrix(axis_x, axis_y, axis_z, angle_degrees):
    length = math.sqrt(axis_x**2 + axis_y**2 + axis_z**2)
    if length == 0.0:
        return _identity_matrix()
    x = axis_x / length
    y = axis_y / length
    z = axis_z / length
    angle = math.radians(angle_degrees)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    one_minus_cosine = 1.0 - cosine
    return (
        cosine + x * x * one_minus_cosine,
        x * y * one_minus_cosine - z * sine,
        x * z * one_minus_cosine + y * sine,
        0.0,
        y * x * one_minus_cosine + z * sine,
        cosine + y * y * one_minus_cosine,
        y * z * one_minus_cosine - x * sine,
        0.0,
        z * x * one_minus_cosine - y * sine,
        z * y * one_minus_cosine + x * sine,
        cosine + z * z * one_minus_cosine,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
    )


def _collada_visual_center(mesh_path):
    tree = ET.parse(mesh_path)
    root = tree.getroot()
    geometries = {}
    for geometry in root.findall(".//c:library_geometries/c:geometry", COLLADA_NS):
        geometry_id = geometry.get("id")
        if not geometry_id:
            continue
        positions = _geometry_positions(geometry)
        if positions:
            geometries[geometry_id] = positions

    transformed_points = []
    visual_scene = root.find(".//c:library_visual_scenes/c:visual_scene", COLLADA_NS)
    if visual_scene is not None:
        for node in visual_scene.findall("./c:node", COLLADA_NS):
            _collect_node_points(
                node,
                _identity_matrix(),
                geometries,
                transformed_points,
            )

    if not transformed_points:
        for points in geometries.values():
            transformed_points.extend(points)
    if not transformed_points:
        raise ValueError(f"Missing Collada position data in {mesh_path}")

    minimum = [min(point[index] for point in transformed_points) for index in range(3)]
    maximum = [max(point[index] for point in transformed_points) for index in range(3)]
    return tuple((minimum[index] + maximum[index]) * 0.5 for index in range(3))


def _geometry_positions(geometry):
    for source in geometry.findall("./c:mesh/c:source", COLLADA_NS):
        source_id = (source.get("id") or "").lower()
        if "position" not in source_id:
            continue
        array = source.find("./c:float_array", COLLADA_NS)
        if array is None:
            continue
        values = [float(value) for value in (array.text or "").split()]
        return list(zip(values[0::3], values[1::3], values[2::3]))
    return []


def _collect_node_points(node, parent_transform, geometries, output):
    transform = _matrix_multiply(parent_transform, _local_transform(node))
    for instance in node.findall("./c:instance_geometry", COLLADA_NS):
        geometry_id = (instance.get("url") or "").lstrip("#")
        for point in geometries.get(geometry_id, []):
            output.append(_matrix_transform(transform, point))

    for child in node.findall("./c:node", COLLADA_NS):
        _collect_node_points(child, transform, geometries, output)
