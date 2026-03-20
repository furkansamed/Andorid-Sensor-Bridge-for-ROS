from setuptools import find_packages, setup


package_name = "vio_stream_bridge"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/vio_stream_bridge.launch.py"]),
        (
            f"share/{package_name}/config",
            ["config/vio_stream_bridge.yaml", "config/cyclonedds.xml"],
        ),
        (f"share/{package_name}", ["README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hp",
    maintainer_email="user@example.com",
    description="ROS 2 bridge for Android VIO video + IMU streams.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "vio_stream_bridge = vio_stream_bridge.bridge_node:main",
        ],
    },
)
