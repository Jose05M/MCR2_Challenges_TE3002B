from setuptools import setup
package_name = 'trajectory_controller'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/square.yaml']),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'trajectory_generator = trajectory_controller.trajectory_generator:main',
            'path_controller = trajectory_controller.path_controller:main',
        ],
    },
)
