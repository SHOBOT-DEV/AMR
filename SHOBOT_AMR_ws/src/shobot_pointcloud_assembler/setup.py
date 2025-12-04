from setuptools import setup

package_name = "shobot_pointcloud_assembler"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/shobot_pointcloud_assembler_launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SHOBOT",
    maintainer_email="todo@example.com",
    description="Placeholder ROS2 package for shobot_pointcloud_assembler (SHOBOT port).",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "shobot_pointcloud_assembler_node = shobot_pointcloud_assembler.node:main",
        ],
    },
)
