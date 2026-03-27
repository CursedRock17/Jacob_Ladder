from setuptools import find_packages, setup

package_name = 'ros2_yolo_image_processing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matrix_Lab',
    maintainer_email='lwendlan@umd.edu',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'drogue_detection_node = ros2_yolo_image_processing.drogue_detection_node:main',
            'pose_estimation_node = ros2_yolo_image_processing.pose_estimation_node:main',
        ],
    },
)
