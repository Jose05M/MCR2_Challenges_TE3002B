from setuptools import setup

package_name = 'pzb_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jose Eduardo Sanchez Martinez',
    maintainer_email='eduardo.mtz1403@gmail.com',
    description='Puzzlebot controllers and path generators',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'open_loop_path_generator = pzb_control.open_loop_path_generator:main',
            'open_loop_controller = pzb_control.open_loop_controller:main',
            'closed_loop_path_generator = pzb_control.closed_loop_path_generator:main',
            'closed_loop_controller = pzb_control.closed_loop_controller:main',
            'traffic_light_nav_controller = pzb_control.traffic_light_nav_controller:main',
            'line_follower_controller = pzb_control.line_follower_controller:main',
        ],
    },
)
