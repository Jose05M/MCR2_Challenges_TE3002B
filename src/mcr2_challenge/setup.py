import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mcr2_challenge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jos',
    maintainer_email='jos@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'traffic_light_detector = mcr2_challenge.traffic_light_detector:main',
            'navigation_controller  = mcr2_challenge.navigation_controller:main',
            'puzzlebot_odometry  = mcr2_challenge.puzzlebot_odometry:main',
            'analysis_node =  mcr2_challenge.analysis_node:main',
        ],
    },
)
