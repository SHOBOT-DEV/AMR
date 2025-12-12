from setuptools import setup

package_name = "shobot_system_monitor"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/system_heartbeat_launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SHOBOT",
    maintainer_email="todo@example.com",
    description="Heartbeat / liveness monitor for SHOBOT (CPU, memory, sensors, nodes).",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "system_heartbeat_node = shobot_system_monitor.heartbeat_node:main",
        ],
    },
)
