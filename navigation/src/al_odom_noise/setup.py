import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'al_odom_noise'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matvey Razgulov',
    maintainer_email='matvey.razgulov@gmail.com',
    description='Drifting noisy odometry from ideal Gazebo planar_move odometry.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_noise_node = al_odom_noise.odom_noise_node:main',
        ],
    },
)
