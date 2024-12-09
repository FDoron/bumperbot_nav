from setuptools import setup
import os
from glob import glob

package_name = 'bumperbot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@email.com',
    description='Navigation package for bumperbot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'f2c_path_planner = bumperbot_navigation.f2c_path_planner:main',
            'path_converter = bumperbot_navigation.path_converter:main',
            # 'nav2_twist_relay = bumperbot_navigation.nav2_twist_relay:main',
            'path_executor = bumperbot_navigation.path_executor:main',
            'waypoint_follower = bumperbot_navigation.waypoint_follower:main',
            'frame_verification = bumperbot_navigation.frame_verification:main',
        ],
    },
)