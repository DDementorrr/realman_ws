from glob import glob
from setuptools import setup

package_name = "pick_place_scene"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ww",
    maintainer_email="ww@example.com",
    description="PlanningScene synchronization for standardized object observations.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "scene_sync = pick_place_scene.scene_sync:main",
    ]},
)
