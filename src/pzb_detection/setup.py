from glob import glob

from setuptools import setup

package_name = 'pzb_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', glob('models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jose Eduardo Sanchez Martinez',
    maintainer_email='eduardo.mtz1403@gmail.com',
    description='Puzzlebot neural-network detection (YOLOv8): traffic signs',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traffic_sign_detector = pzb_detection.traffic_sign_detector:main',
        ],
    },
)
