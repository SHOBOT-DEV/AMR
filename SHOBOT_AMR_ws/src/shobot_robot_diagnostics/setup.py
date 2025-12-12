from setuptools import setup

package_name = "shobot_robot_diagnostics"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/robot_diagnostics_launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SHOBOT",
    maintainer_email="todo@example.com",
    description="Publishes diagnostic summaries (motor temp, IMU, encoders, camera, network) to /diagnostics.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot_diagnostics_node = shobot_robot_diagnostics.robot_diagnostics_node:main",
        ],
    },
)
