#!/usr/bin/env python3
"""Verify that perception publishes observations and scene sync owns writes."""

from pathlib import Path
import ast
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
ADAPTERS = ROOT / "src/ros2_adapters"
PERCEPTION = ADAPTERS / "pick_place_perception"
SCENE = ADAPTERS / "pick_place_scene"
ESTIMATOR = PERCEPTION / "pick_place_perception/aruco_object_pose.py"
RGBD_CONTRACT = PERCEPTION / "pick_place_perception/rgbd_contract.py"
SYNC = SCENE / "pick_place_scene/scene_sync.py"
PERCEPTION_LAUNCH = PERCEPTION / "launch/perception_external.launch.py"
SCENE_LAUNCH = SCENE / "launch/scene_sync_external.launch.py"
LAUNCH = ADAPTERS / "pick_place_sim/launch/vision_pick_place.launch.py"


def fail(message):
    print(f"FAIL {message}")
    raise SystemExit(1)


def manifest(package_dir):
    try:
        return ET.parse(package_dir / "package.xml").getroot()
    except (OSError, ET.ParseError) as exc:
        fail(f"cannot parse {package_dir.name} manifest: {exc}")


for package_dir in (PERCEPTION, SCENE):
    root = manifest(package_dir)
    name = root.findtext("name")
    if name != package_dir.name:
        fail(f"manifest name mismatch in {package_dir}")
    deps = {element.text for element in root if element.tag.endswith("depend") and element.text}
    forbidden = {"gazebo", "robot_gazebo", "pick_place_sim", "ros_gz_sim", "ros_gz_bridge", "gz_ros2_control"}
    leaked = sorted(deps & forbidden)
    if leaked:
        fail(f"{name} has simulation dependencies: {', '.join(leaked)}")

estimator_source = ESTIMATOR.read_text(encoding="utf-8")
rgbd_contract_source = RGBD_CONTRACT.read_text(encoding="utf-8")
if "ApplyPlanningScene" in estimator_source or "create_client" in estimator_source:
    fail("perception estimator must not create scene-writing clients")
if "CollisionObject" not in estimator_source or "/perception/collision_objects" not in estimator_source:
    fail("perception estimator must publish a standard CollisionObject observation")
if "plan_ready" in estimator_source or "scene_frozen" in estimator_source:
    fail("perception freshness monitoring must not stop when the MTC plan becomes ready")
for parameter in ("rgb_image_topic", "depth_image_topic", "camera_info_topic", "depth_encoding",
                  "depth_scale", "max_time_skew"):
    if f'declare_parameter("{parameter}"' not in estimator_source:
        fail(f"perception estimator missing explicit RGB-D parameter: {parameter}")
if '"16UC1"' not in rgbd_contract_source or '"32FC1"' not in rgbd_contract_source:
    fail("perception estimator must support 16UC1 and 32FC1 depth")
if "depth.is_bigendian" not in rgbd_contract_source:
    fail("depth parser must respect Image endianness")

scene_source = SYNC.read_text(encoding="utf-8")
if "ApplyPlanningScene" not in scene_source or "create_client" not in scene_source:
    fail("scene sync must own PlanningScene writes for visual observations")
if "/perception/collision_objects" not in scene_source:
    fail("scene sync must subscribe to the perception observation topic")
if "GetPlanningScene" not in scene_source or "attached_collision_objects" not in scene_source:
    fail("scene sync must protect attached objects from perception world updates")
if "WORLD_OBJECT_GEOMETRY" not in scene_source or "geometry_matches" not in scene_source:
    fail("scene sync must query back and confirm successfully applied world objects")
if "client.service_is_ready()" not in scene_source:
    fail("scene sync must fail closed when PlanningScene apply service is unavailable")
if "obj.id in self.transactions" not in scene_source or "math.isfinite" not in scene_source:
    fail("scene sync must suppress duplicate in-flight updates and reject non-finite geometry")
if "ROBOT_STATE_ATTACHED_OBJECTS" not in scene_source or "ClockType.STEADY_TIME" not in scene_source:
    fail("scene sync must request attachments and use a steady-clock timeout")

for entry, config_arg in ((PERCEPTION_LAUNCH, "perception_config"), (SCENE_LAUNCH, "scene_config")):
    entry_source = entry.read_text(encoding="utf-8")
    if "gazebo" in entry_source.lower() or "robot_gazebo" in entry_source or "IncludeLaunchDescription" in entry_source:
        fail(f"independent entry {entry.name} must not start simulation or include another launch")
    if config_arg not in entry_source or "use_sim_time" not in entry_source:
        fail(f"independent entry {entry.name} must expose config and clock choice")

launch_source = LAUNCH.read_text(encoding="utf-8")
for required in ("pick_place_perception", "pick_place_scene", "aruco_object_pose", "scene_sync"):
    if required not in launch_source:
        fail(f"vision launch missing separated node wiring: {required}")
if "package=\"pick_place_demo\", executable=\"aruco_object_pose\"" in launch_source:
    fail("visual flow must launch the extracted estimator package")
for value in ("32FC1", "1.0", "0.1", "/camera/camera/color/image_raw",
              "/camera/camera/depth/image_rect_raw", "/camera/camera/color/camera_info"):
    if value not in launch_source:
        fail(f"vision launch does not explicitly preserve RGB-D contract value {value}")

demo_manifest = manifest(ADAPTERS / "pick_place_demo")
if "pick_place_perception" not in {e.text for e in demo_manifest if e.tag.endswith("depend")}:
    fail("compatibility package must depend on the extracted perception package")
sim_manifest = manifest(ADAPTERS / "pick_place_sim")
sim_deps = {e.text for e in sim_manifest if e.tag.endswith("depend")}
if not {"pick_place_perception", "pick_place_scene"} <= sim_deps:
    fail("pick_place_sim must declare runtime dependencies for both split nodes")

print("PASS perception/scene package ownership, manifests, and visual launch wiring")
