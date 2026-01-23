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
        # Include message files
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Tennis ball machine controller',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'arduino_controller_node = tennis_ball_machine.arduino_controller_node:main',
            'ard_controller_simple = tennis_ball_machine.ard_controller_simple:main', 
            'person_tracker_node = tennis_ball_machine.person_tracker_node:main',
        ],
    },
)
