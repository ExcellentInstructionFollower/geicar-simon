from setuptools import find_packages, setup

package_name = 'pose_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GeiCar maintainers',
    maintainer_email='a.pierreditlambert@gmail.com',
    description='ROS 2 bridge to TensorRT YOLO-Pose inference and human motion detection.',
    license='GPL-2.0-only',
    entry_points={
        'console_scripts': [
            'pose_bridge_node = pose_vision.pose_bridge_node:main',
            'motion_detector_node = pose_vision.motion_detector_node:main',
        ],
    },
)
