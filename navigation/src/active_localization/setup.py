from glob import glob

from setuptools import setup

package_name = 'active_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + '.scorers'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matvey Razgulov',
    maintainer_email='matvey.razgulov@gmail.com',
    description='Localization-aware navigation on top of Nav2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'al_navigator = active_localization.navigator_node:main',
            'al_cost_field = active_localization.cost_field_node:main',
        ],
    },
)
