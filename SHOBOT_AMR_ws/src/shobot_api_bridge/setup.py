from setuptools import setup

package_name = "shobot_api_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/api_bridge_launch.py"]),
    ],
    install_requires=["setuptools", "flask", "flask-cors"],
    zip_safe=True,
    maintainer="SHOBOT",
    maintainer_email="todo@example.com",
    description="Flask-based REST bridge for SHOBOT to interact with ROS2 topics.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "api_bridge = shobot_api_bridge.api_bridge:main",
        ],
    },
)
