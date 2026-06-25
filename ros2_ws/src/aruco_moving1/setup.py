from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'aruco_moving'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='storagy',
    maintainer_email='storagy@todo.todo',
    description='ArUco 마커 인식 및 도킹 제어 패키지',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_moving = aruco_moving.aruco_moving:main',
            'aruco_detector = aruco_moving.aruco_detector:main',
        ],
    },
)
