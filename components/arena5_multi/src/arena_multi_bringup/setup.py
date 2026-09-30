from glob import glob
import os
from setuptools import find_packages, setup

package_name = "arena_multi_bringup"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "behavior_trees"), glob("behavior_trees/*.xml")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="lpc-robotics",
    maintainer_email="lpc-robotics@users.noreply.github.com",
    description="Arena5 multi-robot bringup",
    license="Apache-2.0",
    entry_points={"console_scripts": ["scenario_manager = arena_multi_bringup.scenario_manager:main"]},
)
