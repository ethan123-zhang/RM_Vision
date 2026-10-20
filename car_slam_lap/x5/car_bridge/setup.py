from setuptools import setup

package_name = "car_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyserial", "PyYAML"],
    zip_safe=True,
    maintainer="RM Vision",
    maintainer_email="rm@example.com",
    description="X5 <-> C8T6 串口底盘桥接与建图/回放工具",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "bridge_node = car_bridge.bridge_node:main",
            "link_test = car_bridge.link_test:main",
            "teleop_keys = car_bridge.teleop_keys:main",
            "record_path = car_bridge.record_path:main",
            "follow_path = car_bridge.follow_path:main",
            "map_from_cloud = car_bridge.map_from_cloud:main",
            "calibrate_speed = car_bridge.calibrate_speed:main",
        ],
    },
)
