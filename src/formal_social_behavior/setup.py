from glob import glob

from setuptools import find_packages, setup


package_name = "formal_social_behavior"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="social-nav-x maintainers",
    maintainer_email="noreply@example.com",
    description=(
        "Deterministic social-state automata and HuNav v1 reset proxy "
        "for Arena Isaac"
    ),
    license="MIT",
    entry_points={
        "console_scripts": [
            "formal_social_behavior_proxy = "
            "formal_social_behavior.proxy_node:main",
            "verify_formal_social_scenario = "
            "formal_social_behavior.scenario_verifier:main",
            "formal_social_multi_proxy = "
            "formal_social_behavior.multi_agent.proxy_node:main",
            "export_formal_social_multi_model = "
            "formal_social_behavior.multi_agent.model_export:main",
            "replay_formal_social_multi = "
            "formal_social_behavior.multi_agent.replay:main",
            "verify_formal_social_multi_scenario = "
            "formal_social_behavior.multi_agent.scenario_verifier:main",
        ],
    },
)
