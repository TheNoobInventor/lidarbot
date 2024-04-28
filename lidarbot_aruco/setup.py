import os
from glob import glob
from setuptools import find_packages, setup

package_name = "lidarbot_aruco"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*yaml")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="noobinventor",
    maintainer_email="eduobijoro@mailfence.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "aruco_trajectory_visualizer_node = lidarbot_aruco.aruco_trajectory_visualizer.main"
        ],
    },
)
