from setuptools import find_packages, setup

package_name = "jl_flight"

setup(
    name=package_name,
    version="0.2.0",
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
    description="Fly missions written as plain Python.",
    license="BSD-3-Clause",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "readiness = jl_flight.ros.readiness_probe:main",
            "setpoint_timing = jl_flight.ros.setpoint_timing:main",
        ],
    },
)
