import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'canbus_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rafiq Rushdan',
    maintainer_email='rafiqkizars21@gmail.com',
    description='This package is used for testing canbus protocol with MKS42C PCBA Nema17',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_canbus = canbus_ros.motor_canbus:main'
        ],
    },
)
