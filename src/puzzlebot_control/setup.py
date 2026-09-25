from setuptools import find_packages, setup

package_name = 'puzzlebot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jos',
    maintainer_email='jos@todo.todo',
    description='Controlador y generador de trayectorias para el Puzzlebot',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'path_generator = puzzlebot_control.path_generator:main',
            'controller_node = puzzlebot_control.controller_node:main',
            'puzzlebot_odometry = puzzlebot_control.puzzlebot_odometry:main',
            'square_node = puzzlebot_control.square_node:main'
        ],
    },
)
