from setuptools import setup
import os
from glob import glob

package_name = 'tennis_ball_machine'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Padmanabha Kavasseri',
    maintainer_email='pkavasseri@gmail.com',
    description='Tennis ball machine controller',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ard_controller_node = tennis_ball_machine.ard_controller_node:main', 
            'person_tracker_node = tennis_ball_machine.person_tracker_node:main',
        ],
    },
)
