#!/usr/bin/env python3
"""Check that the external MTC launch does not own simulation or scene setup."""

from pathlib import Path
import ast
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "src/ros2_adapters/pick_place_mtc/launch/plan_pick_place_external.launch.py"
PACKAGE = ROOT / "src/ros2_adapters/pick_place_mtc"
PUBLIC_LAUNCH = PACKAGE / "launch/plan_pick_place.launch.py"


def fail(message: str) -> None:
    print(f"FAIL {message}")
    raise SystemExit(1)


try:
    source = LAUNCH.read_text(encoding="utf-8")
    tree = ast.parse(source, filename=str(LAUNCH))
except (OSError, SyntaxError) as exc:
    fail(f"cannot parse external planning launch: {exc}")

if "IncludeLaunchDescription" in source:
    fail("external planning launch must not include another launch")

for forbidden in ("gazebo", "robot_gazebo", "pick_place_demo", "known_scene", "pick_place_sim",
                 "ros_gz", "controller", "camera"):
    if forbidden in source.lower():
        fail(f"external planning launch references forbidden simulation/scene term: {forbidden}")

declared = {
    node.value
    for node in ast.walk(tree)
    if isinstance(node, ast.Constant) and isinstance(node.value, str)
}
if "planning_config" not in declared or "use_sim_time" not in declared:
    fail("planning_config and use_sim_time must be explicit launch arguments")
if "plan_pick_place" not in declared or "pick_place_mtc" not in declared:
    fail("launch must start the pick_place_mtc plan_pick_place executable")
if "trajectory_execution.allowed_execution_duration_scaling" in source:
    fail("simulation-specific controller timeout override must not be set")

planner_source = (PACKAGE / "src/plan_pick_place.cpp").read_text(encoding="utf-8")
for required in (
        '"/perception/scene_revision"',
        '"/perception/scene_ready"',
        '"/pick_place/candidate_scene_revision"',
        "scene_revision_seen_",
        "candidate_revision_pub_"):
    if required not in planner_source:
        fail(f"planner must bind candidates to SceneSync revision/ready: missing {required}")

node_calls = [
    node for node in ast.walk(tree)
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "Node"
]
if len(node_calls) != 1:
    fail(f"expected exactly one launch Node, found {len(node_calls)}")

print("PASS external MTC launch has explicit config/time and no simulation or scene provider")

public_source = PUBLIC_LAUNCH.read_text(encoding="utf-8")
public_tree = ast.parse(public_source, filename=str(PUBLIC_LAUNCH))
declared_args = {}
for node in ast.walk(public_tree):
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "DeclareLaunchArgument":
        name = node.args[0].value if node.args and isinstance(node.args[0], ast.Constant) else None
        declared_args[name] = any(keyword.arg == "default_value" for keyword in node.keywords)
for required in ("planning_config", "task_config"):
    if declared_args.get(required, True):
        fail(f"public launch must require explicit {required}")
if declared_args.get("use_sim_time") is not True:
    fail("public launch must expose use_sim_time")
if "plan_pick_place_external.launch.py" not in public_source:
    fail("public launch must reuse the external-only entry")

try:
    manifest = ET.parse(PACKAGE / "package.xml").getroot()
except (OSError, ET.ParseError) as exc:
    fail(f"cannot parse MTC manifest: {exc}")
dependency_tags = {"depend", "build_depend", "build_export_depend", "exec_depend", "buildtool_depend"}
dependencies = {element.text.strip() for element in manifest
                if element.tag in dependency_tags and element.text}
forbidden_dependencies = {
    "gz_ros2_control", "moveit_configs_utils", "pick_place_demo", "pick_place_sim",
    "robot_gazebo", "robot_realman_moveit_config", "ros_gz_bridge", "ros_gz_sim",
}
leaked = dependencies & forbidden_dependencies
if leaked:
    fail(f"MTC package declares simulation/model demo dependencies: {', '.join(sorted(leaked))}")
for required in ("ament_index_python", "launch", "launch_ros"):
    if required not in dependencies:
        fail(f"MTC public launch is missing dependency {required}")
print("PASS public planning launch and package manifest are externally configured and simulation independent")
