from setuptools import find_packages, setup

package_name = 'wall_follower_pid'

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
    maintainer='serga',
    maintainer_email='serga@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_distance_finder = wall_follower_pid.wall_distance_finder:main',
            'control = wall_follower_pid.control:main',
        ],
    },
)