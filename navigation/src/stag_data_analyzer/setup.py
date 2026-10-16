from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'stag_data_analyzer'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'pyyaml',
        'numpy',
        'plotly',
        'kaleido',  # for saving plotly figures as static images
    ],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Data analysis and visualization package for STag marker detection experiments',
    license='MIT',
    entry_points={
        'console_scripts': [
            'analyze_data = stag_data_analyzer.main:main',
        ],
    },
)
