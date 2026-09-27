from setuptools import setup

package_name = 'pzb_fsm'

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
    description='Puzzlebot decision making: FSM for traffic lights, signs and intersections',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fsm_node = pzb_fsm.fsm_node:main',
        ],
    },
)
