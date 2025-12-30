from setuptools import setup

package_name = "shobot_rl_agent"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/shobot_rl_agent_launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="todo@example.com",
    description="RL policy runner node for SHOBOT",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "shobot_rl_agent = shobot_rl_agent:main",
            "shobot_rl_intent = shobot_rl_intent:main",
            "shobot_rl_speed = shobot_rl_speed:main",
        ],
    },
)
