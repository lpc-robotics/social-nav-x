from setuptools import find_packages, setup

package_name = "arena_multi_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="lpc-robotics",
    maintainer_email="lpc-robotics@users.noreply.github.com",
    description="Arena5 multi-robot runtime tools",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "command_guard = arena_multi_control.command_guard:main",
            "wheel_visualizer = arena_multi_control.wheel_visualizer:main",
            "multi_nav_goal = arena_multi_control.task_cli:goal_main",
            "multi_nav_waypoints = arena_multi_control.task_cli:waypoints_main",
            "multi_nav_cancel = arena_multi_control.task_cli:cancel_main",
        ]
    },
)
