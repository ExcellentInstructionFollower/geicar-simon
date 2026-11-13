from setuptools import find_packages, setup

package_name = 'obstacle_stop'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GeiCar maintainers',
    maintainer_email='a.pierreditlambert@gmail.com',
    description='Stops the vehicle when the lidar detects a nearby obstacle.',
    license='GPL-2.0-only',
    entry_points={
        'console_scripts': [
            'obstacle_stop_node = obstacle_stop.obstacle_stop_node:main',
        ],
    },
)
