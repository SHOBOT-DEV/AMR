from setuptools import setup

package_name = "shobot_sensors"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/shobot_sensors_launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SHOBOT",
    maintainer_email="todo@example.com",
    description="IMU republisher and wheel encoder odometry for SHOBOT.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "imu_republisher_node = shobot_sensors.imu_republisher:main",
            "wheel_odometry_node = shobot_sensors.wheel_odometry:main",
            "battery_monitor_node = shobot_sensors.battery_monitor:main",
        ],
    },
)
