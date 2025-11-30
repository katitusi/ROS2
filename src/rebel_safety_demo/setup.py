from setuptools import setup
import os
from glob import glob

package_name = 'rebel_safety_demo'

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
    description='Safety demo for igus ReBeL robot with optional LLM supervisor',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rebel_human_distance_publisher = rebel_safety_demo.human_distance_publisher:main',
            'rebel_mover = rebel_safety_demo.rebel_mover:main',
            'llm_safety_supervisor = rebel_safety_demo.llm_safety_supervisor:main',
        ],
    },
)
