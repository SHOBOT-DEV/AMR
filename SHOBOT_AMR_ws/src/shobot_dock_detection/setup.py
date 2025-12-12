from setuptools import setup

package_name = "shobot_dock_detection"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/dock_detection_launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SHOBOT",
    maintainer_email="todo@example.com",
    description="Dock station detection aggregator (AprilTag/QR/ArUco/etc.) publishing unified dock pose.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dock_detection_node = shobot_dock_detection.dock_detection_node:main",
        ],
    },
)
