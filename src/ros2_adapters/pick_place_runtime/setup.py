from setuptools import setup

package_name = "pick_place_runtime"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ww",
    maintainer_email="ww@example.com",
    description="Backend-independent pick-and-place execution protocol.",
    license="Apache-2.0",
)
