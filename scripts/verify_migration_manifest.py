#!/usr/bin/env python3
"""Static checks for the deliberately small RM65B migration repository."""

from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
EXPECTED_COMMIT = "c941b565e4f9174afa36561f143ef5fbbb744750"
EXPECTED_GZ_CONTROL_COMMIT = "522ca0e0d3560dc8c52628e9f52b86db78673920"
REQUIRED_PACKAGES = {
    "robot_changingtek_description",
    "robot_realman_moveit_config",
    "robot_gazebo",
    "manipulation_bringup",
    "manipulation_moveit_adapter",
    "rm65b_operator_panel",
    "manipulation_demo",
    "changingtek_gripper_driver",
}
BACKEND_INDEPENDENT_PACKAGES = {
    "pick_place_perception",
    "pick_place_scene",
    "pick_place_runtime",
}
FORBIDDEN_BACKEND_DEPENDENCIES = {
    "gazebo",
    "robot_gazebo",
    "ros_gz_sim",
    "ros_gz_bridge",
    "gz_ros2_control",
    "pick_place_sim",
}
FORBIDDEN_PATH_PARTS = {"manipulation_core", "franka_description", "moveit_resources"}
DEPENDENCY_TAGS = {
    "depend",
    "build_depend",
    "build_export_depend",
    "buildtool_depend",
    "exec_depend",
    "run_depend",
    "test_depend",
    "doc_depend",
}


def fail(message: str) -> None:
    print(f"FAIL {message}")
    raise SystemExit(1)


for package in sorted(REQUIRED_PACKAGES):
    package_roots = list((ROOT / "src").rglob("package.xml"))
    package_xml = next((p for p in package_roots if f"<name>{package}</name>" in p.read_text(encoding="utf-8")), None)
    if package_xml is None:
        fail(f"missing migrated package {package}")
    print(f"PASS migrated package {package}")

for package in sorted(BACKEND_INDEPENDENT_PACKAGES):
    candidates = [
        p
        for p in sorted((ROOT / "src").rglob("package.xml"))
        if p.parent.name == package
        and f"<name>{package}</name>" in p.read_text(encoding="utf-8")
    ]
    if not candidates:
        fail(f"missing reusable package {package}")
    package_xml = candidates[0]
    try:
        root = ET.parse(package_xml).getroot()
    except ET.ParseError as error:
        fail(f"cannot parse manifest {package_xml.relative_to(ROOT).as_posix()}: {error}")
    if (root.findtext("name") or "").strip() != package:
        fail(f"manifest {package_xml.relative_to(ROOT).as_posix()} does not declare package name {package}")
    declared = set()
    for element in root:
        if element.tag in DEPENDENCY_TAGS and element.text:
            declared.add(element.text.strip())
    forbidden = sorted(declared & FORBIDDEN_BACKEND_DEPENDENCIES)
    if forbidden:
        fail(
            f"reusable package {package} must stay simulation-independent but "
            f"{package_xml.relative_to(ROOT).as_posix()} depends on {', '.join(forbidden)}"
        )
    print(f"PASS reusable package {package} has no simulation-only dependencies")

for required in (
    "src/realman_rm65/robot_gazebo/urdf/rm65b_ctm2f110_gz.urdf.xacro",
    "src/realman_rm65/robot_gazebo/urdf/rm65b_gz.urdf.xacro",
    "src/realman_rm65/robot_gazebo/worlds/empty_mimic.sdf",
    "src/ros2_adapters/manipulation_bringup/launch/realman_moveit_gz.launch.py",
):
    if not (ROOT / required).is_file():
        fail(f"missing required migrated runtime asset {required}")
print("PASS RM65B gripper and bare-arm fallback runtime assets")

for required in (
    "src/crt_ctm2f110_gripper_visualization/changingtek_gripper_driver/vendor/ctm2f110_sdk/gripper.cpp",
    "src/crt_ctm2f110_gripper_visualization/changingtek_gripper_driver/vendor/ctm2f110_sdk/include/CRTS.h",
    "src/crt_ctm2f110_gripper_visualization/changingtek_gripper_driver/vendor/ctm2f110_sdk/include/x86_64/libCRTServo.so",
    "src/crt_ctm2f110_gripper_visualization/changingtek_gripper_driver/vendor/ctm2f110_sdk/include/arm64/libCRTServo.so",
):
    if not (ROOT / required).is_file():
        fail(f"missing vendored CTM2F110 SDK asset {required}")
print("PASS vendored CTM2F110 SDK assets for x86_64 and arm64")

link = ROOT / "src" / "realman_rm65" / "rm_description"
if not link.is_symlink():
    fail("src/realman_rm65/rm_description must remain a symlink to imported vendor source")
print("PASS rm_description is a vendor-source symlink")

manifest = (ROOT / "upstream" / "realman_humble.repos").read_text(encoding="utf-8")
if EXPECTED_COMMIT not in manifest:
    fail("RealMan Humble manifest does not contain the expected full commit")
print(f"PASS RealMan source commit pin {EXPECTED_COMMIT}")

gz_manifest = (ROOT / "upstream" / "gz_ros2_control_humble.repos").read_text(encoding="utf-8")
if EXPECTED_GZ_CONTROL_COMMIT not in gz_manifest:
    fail("gz_ros2_control Harmonic source is not pinned to the expected commit")
print(f"PASS gz_ros2_control Harmonic commit pin {EXPECTED_GZ_CONTROL_COMMIT}")

for path in (ROOT / "src").rglob("*"):
    if not path.is_file() or path.is_symlink():
        continue
    relative = path.relative_to(ROOT).as_posix()
    if any(part in relative for part in FORBIDDEN_PATH_PARTS):
        fail(f"official or educational excluded content leaked into migration: {relative}")
    text = path.read_text(encoding="utf-8", errors="ignore")
    for forbidden, label in (
        ("/opt/ros/jazzy", "Jazzy absolute path"),
        ("/home/ww/manipulation_ws", "source workspace absolute path"),
    ):
        if forbidden in text:
            fail(f"{label} found in {relative}")

print("PASS migrated source has no Jazzy/source-workspace references")
print("migration-manifest-check: PASS")
