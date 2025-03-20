import os
from glob import glob
from setuptools import setup

package_name = 'avionics_bms'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),  # Package index marker
        ('share/' + package_name, ['package.xml']),  # Package metadata
        ('share/' + package_name + '/launch', glob('launch/*.py')),  # Install launch files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matas',
    maintainer_email='matas.jones@epfl.ch',
    description='BMS publishing node: BMS -> ROS',
    license='-',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bms_publisher = avionics_bms.bms_monitor:main',
        ],
    },
)



