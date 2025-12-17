from setuptools import setup

package_name = "shobot_costmap_plugins"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/shobot_costmap_plugins_launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SHOBOT",
    maintainer_email="todo@example.com",
    description="PointCloud-to-OccupancyGrid costmap helper for SHOBOT.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "shobot_costmap_plugins_node = shobot_costmap_plugins.node:main",
        ],
    },
)
