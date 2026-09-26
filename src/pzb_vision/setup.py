from setuptools import setup

package_name = 'pzb_vision'

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
    description='Puzzlebot computer vision (OpenCV): traffic light and line detection',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traffic_light_detector = pzb_vision.traffic_light_detector:main',
            'line_detector = pzb_vision.line_detector:main',
            'usb_camera_publisher = pzb_vision.usb_camera_publisher:main',
        ],
    },
)
