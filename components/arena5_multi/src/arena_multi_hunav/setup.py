from setuptools import find_packages, setup

package_name = "arena_multi_hunav"
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
    maintainer="lpc-robotics",
    maintainer_email="lpc-robotics@users.noreply.github.com",
    description="Limited HuNav multi-robot adapter",
    license="Apache-2.0",
    entry_points={"console_scripts": ["hunav_multi_adapter = arena_multi_hunav.adapter:main",
                                      "multi_sfm_adapter = arena_multi_hunav.multi_adapter:main"]},
)
