from setuptools import setup
import os
from glob import glob

package_name = 'rebel_dance_demo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROS2 Developer',
    maintainer_email='user@example.com',
    description='30-second dance choreography for igus ReBeL robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rebel_dancer = rebel_dance_demo.rebel_dancer:main',
        ],
    },
)
