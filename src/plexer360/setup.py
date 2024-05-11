from setuptools import find_packages, setup

package_name = 'plexer360'

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
            'sub = src.field_view:main',
            'pub = src.control:main',
            'dec = src.multiplexer:main',
            'learn = src.dqn_bridge:main',
        ],
    },
)