from glob import glob
from setuptools import setup

package_name = "pick_place_perception"

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
    description="Backend-independent RGB-D target pose and obstacle geometry.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "aruco_object_pose = pick_place_perception.aruco_object_pose:main",
        "depth_obstacles = pick_place_perception.depth_obstacles:main",
    ]},
)
