from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'waypoint_follow'

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
    maintainer='htetwaiyan',
    maintainer_email='htetwaiyan.zyme@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_follow = waypoint_follow.waypoint_follow:main',
            'waypoint_viasualizer = waypoint_follow.waypoint_viasualizer:main',
        ],
    },
)
