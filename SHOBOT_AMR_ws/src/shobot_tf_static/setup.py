from setuptools import setup

package_name = "shobot_tf_static"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/static_tf_publisher_launch.py"]),
        ("share/" + package_name + "/config", ["config/static_transforms.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SHOBOT",
    maintainer_email="todo@example.com",
    description="Static TF2 broadcaster for SHOBOT frames.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "static_tf_publisher_node = shobot_tf_static.static_tf_publisher_node:main",
        ],
    },
)
