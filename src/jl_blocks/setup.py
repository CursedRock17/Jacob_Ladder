from setuptools import find_packages, setup

package_name = "jl_blocks"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/mission.launch.py"]),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="Lucas Wendland",
    maintainer_email="mtglucas1@gmail.com",
    description="Compose autonomous missions from reusable Python blocks.",
    license="BSD-3-Clause",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "jl_blocks = jl_blocks.cli:main",
            "mission_runner = jl_blocks.ros.mission_runner:main",
            "mission_watch = jl_blocks.ros.mission_watch:main",
            "readiness = jl_blocks.ros.readiness_probe:main",
            "setpoint_timing = jl_blocks.ros.setpoint_timing:main",
        ],
    },
)
