from setuptools import setup

package_name = "shobot_mission_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/shobot_mission_control_launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SHOBOT",
    maintainer_email="todo@example.com",
    description="Mission queue executor that sends Nav2 goals and optional docking.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "shobot_mission_control_node = shobot_mission_control.node:main",
        ],
    },
)
