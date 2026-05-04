from setuptools import find_packages, setup

package_name = 'oak_d_visual_odometry'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lucas Wendland',
    maintainer_email='mtglucas1@gmail.com',
    description='OAK-D Lite stereo-VIO bridge to PX4 vehicle_visual_odometry.',
    license='TODO',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'rgb_publisher = oak_d_visual_odometry.rgb_publisher:main',
            'vo_publisher_node = oak_d_visual_odometry.vo_publisher_node:main',
            'vo_publisher_px4_node = oak_d_visual_odometry.vo_publisher_px4_node:main',
        ],
    },
)
