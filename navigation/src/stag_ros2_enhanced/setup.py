from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'stag_ros2_enhanced'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (f'share/{package_name}/config', glob('config/*.yaml')),
        (f'share/{package_name}/scripts', glob('scripts/*.py') + glob('scripts/*.sh') + glob('scripts/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Professional STag marker detection and generation for ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            # Executables and scripts are installed by CMakeLists.txt (ament_cmake package)
        ],
    },
)
