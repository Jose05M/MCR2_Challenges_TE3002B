from setuptools import setup

package_name = 'pzb_tools'

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
    description='Puzzlebot data logging and analysis tools',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'analysis_node = pzb_tools.analysis_node:main',
        ],
    },
)
