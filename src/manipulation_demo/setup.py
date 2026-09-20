from setuptools import find_packages, setup

package_name = "manipulation_demo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ww",
    maintainer_email="ww@example.com",
    description="Interactive target pose publisher",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "interactive_target = manipulation_demo.interactive_target:main",
        ]
    },
)
