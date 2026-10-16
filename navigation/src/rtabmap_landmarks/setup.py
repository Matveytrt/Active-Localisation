import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'rtabmap_landmarks'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dmitrytorov',
    maintainer_email='torovdmitrii@gmail.com',
    description='RTAB-Map landmarks helpers (Python)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'landmarks_yaml_markers_node = rtabmap_landmarks.landmarks_yaml_markers_node:main',
            'export_landmarks_from_db = rtabmap_landmarks.export_landmarks_from_db:main',
        ],
    },
)
