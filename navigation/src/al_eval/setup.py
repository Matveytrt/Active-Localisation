from glob import glob

from setuptools import setup

package_name = 'al_eval'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/scenarios',
         glob('config/scenarios/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matvey Razgulov',
    maintainer_email='matvey.razgulov@gmail.com',
    description='TUM trajectory recorder and Nav2 scenario runner for APE/RPE (evo)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traj_recorder = al_eval.traj_recorder_node:main',
            'scenario_runner = al_eval.scenario_runner_node:main',
        ],
    },
)
