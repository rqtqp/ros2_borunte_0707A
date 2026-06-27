from setuptools import find_packages, setup

package_name = "borunte0707a_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rqtqp",
    maintainer_email="a@supb.org",
    description="ROS 2 driver for the Borunte BRTIRUS0707A (HC1) over JSON/TCP.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joint_state_publisher = borunte0707a_driver.joint_state_publisher_node:main",
            "status_node = borunte0707a_driver.status_node:main",
            "motion_bridge = borunte0707a_driver.motion_bridge_node:main",
            "calibration_helper = borunte0707a_driver.calibration_helper:main",
            "kin_calibrate = borunte0707a_driver.kin_calibrate:main",
        ],
    },
)
