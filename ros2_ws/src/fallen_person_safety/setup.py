from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'fallen_person_safety'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='storagy',
    maintainer_email='storagy@example.com',
    description='Fallen-person safety monitoring for the Storagy robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'safety_controller = fallen_person_safety.safety_controller:main',
            'admin_monitor = fallen_person_safety.admin_monitor:main',
            'velocity_mux = fallen_person_safety.velocity_mux:main',
        ],
    },
)
