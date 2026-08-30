from setuptools import setup


package_name = "arena_humble_compat"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="Arena local deployment",
    maintainer_email="noreply@example.com",
    description="Arena Humble to Isaac Sim 5.1 compatibility helpers",
    license="MIT",
    entry_points={
        "console_scripts": [
            "hunav_six_behaviors_bridge = arena_humble_compat.hunav_six_behaviors_bridge:main",
            "scene_bridge = arena_humble_compat.scene_bridge:main",
            "verify_six_behaviors = arena_humble_compat.verify_six_behaviors:main",
            "verify_runtime = arena_humble_compat.verify_runtime:main",
        ],
    },
)
