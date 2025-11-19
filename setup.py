from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'pybuttoning'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'torch>=2.0.0',
        'torchvision>=0.15.0',
        'numpy>=1.21.0',
        'opencv-python>=4.5.0',
        'scipy>=1.7.0',
        'mediapipe>=0.10.0',
        'cloudpickle>=2.0.0',
        # 'detectron2>=0.6',  # Install separately - see README
        # 'kortex_api>=2.6.0',  # Install from Kinova SDK - see README
    ],
    zip_safe=True,
    maintainer='Ha ves',
    maintainer_email='haves@tekat.my.id',
    description='Python ROS2 package for dual-arm buttoning system',
    license='AGPL-3.0-or-later',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_node = pybuttoning.nodes.detection_node:main',
            'hand_detection_node = pybuttoning.nodes.hand_detection_node:main',
            'arm_controller_node = pybuttoning.nodes.arm_controller_node:main',
            'buttoning_controller_node = pybuttoning.nodes.buttoning_controller_node:main',
        ],
    },
)
