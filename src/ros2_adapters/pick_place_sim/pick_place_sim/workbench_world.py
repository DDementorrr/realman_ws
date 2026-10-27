"""Generate a temporary fixed-attachment world from neutral workbench boxes."""

from pathlib import Path
import hashlib
import math
import xml.etree.ElementTree as ET

from pick_place_scene.static_geometry import load_geometry


def prepare_rgbd_fixture(world_file):
    """Remove legacy nonphysical decorations only from a generated RGB-D world."""
    path = Path(world_file)
    original = path.read_bytes()
    tree = ET.fromstring(original)
    target = tree.find("world/model[@name='pick_object']/link[@name='body']")
    marker = tree.find("world/model[@name='place_zone']")
    if target is None or marker is None or marker.findall('.//collision'):
        raise ValueError('RGB-D fixture requires the known visual-only place marker')
    removed = []
    for model_name, links in (('pick_object', [target]), ('place_zone', marker.findall('link'))):
        for link in links:
            for visual in list(link.findall('visual')):
                if model_name == 'place_zone' or visual.get('name', '').startswith('aruco'):
                    removed.append({'model': model_name, 'link': link.get('name'),
                                    'name': visual.get('name'), 'pose': visual.findtext('pose'),
                                    'sdf': ET.tostring(visual, encoding='unicode')})
                    link.remove(visual)
    ET.ElementTree(tree).write(path, encoding='utf-8', xml_declaration=True)
    return {
        'target_source': 'independently known task pose; fiducial detector disabled',
        'removed_noncollision_visuals': removed,
        'collision_geometry_changed': False,
        'original_world_sha256': hashlib.sha256(original).hexdigest(),
        'rgbd_world_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
    }


def generate_world(template, geometry, robot_mount, destination):
    frame, boxes = load_geometry(geometry)
    if not {"pick_table", "left_table", "right_table"} <= {box.id for box in boxes}:
        raise ValueError("workbench requires pick_table, left_table and right_table")
    mount = ET.parse(robot_mount).find("joint[@name='world_to_rm65b']")
    if (mount is None or mount.find("parent").get("link") != "world"
            or mount.find("child").get("link") != frame):
        raise ValueError("geometry frame must match the explicit world-to-robot mount")
    origin = mount.find("origin")
    if any(float(x) != 0 for x in origin.get("rpy", "0 0 0").split()):
        raise ValueError("world adapter requires an unrotated robot mount")
    offset = tuple(float(x) for x in origin.get("xyz").split())
    if len(offset) != 3 or not all(math.isfinite(x) for x in offset):
        raise ValueError("robot mount translation requires three finite numbers")
    tree = ET.parse(template)
    world = tree.find("world")
    if world is None:
        raise ValueError("template must contain one world")
    ids = {box.id for box in boxes}
    reserved = {model.get("name") for model in world.findall("model")} - {
        "pick_table", "left_table", "right_table"}
    if ids & reserved:
        raise ValueError("geometry must not replace template entities: " + str(sorted(ids & reserved)))
    for model in list(world.findall("model")):
        if model.get("name") in ids:
            world.remove(model)
    for box in boxes:
        model = ET.SubElement(world, "model", name=box.id)
        ET.SubElement(model, "static").text = "true"
        pose = tuple(x + t for x, t in zip(box.pose[:3], offset)) + box.pose[3:]
        ET.SubElement(model, "pose").text = " ".join(str(x) for x in pose)
        link = ET.SubElement(model, "link", name="link")
        for kind in ("visual", "collision"):
            shape = ET.SubElement(link, kind, name=kind)
            size = ET.SubElement(ET.SubElement(ET.SubElement(shape, "geometry"), "box"), "size")
            size.text = " ".join(str(x) for x in box.size)
            if kind == "visual":
                material = ET.SubElement(shape, "material")
                ET.SubElement(material, "ambient").text = "0.45 0.45 0.45 1"
                ET.SubElement(material, "diffuse").text = "0.55 0.55 0.55 1"
    tree.write(Path(destination), encoding="utf-8", xml_declaration=True)
    return Path(destination)
