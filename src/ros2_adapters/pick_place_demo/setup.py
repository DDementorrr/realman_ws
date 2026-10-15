from glob import glob
from setuptools import setup

package_name = "pick_place_demo"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ww",
    maintainer_email="ww@example.com",
    description="Known-pose RM65B pick-and-place simulation scene.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "known_scene = pick_place_demo.known_scene:main",
        "aruco_object_pose = pick_place_demo.aruco_object_pose:main",
        "wrist_observation = pick_place_demo.wrist_observation:main",
    ]},
)
