from setuptools import setup
import os
from glob import glob

package_name = 'avp_managers'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/launch', ['launch/avp_managers_launch.py']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zubair Islam',
    maintainer_email='zubxxr@gmail.com',
    description='Manager nodes for AVP system (queue, reservation, vehicle count, etc.)',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drop_off_zone_queue_manager = avp_managers.drop_off_zone_queue_manager:main',
            'parking_spot_reservation_manager = avp_managers.parking_spot_reservation_manager:main',
            'vehicle_count_manager = avp_managers.vehicle_count_manager:main',
            'vehicle_status_manager = avp_managers.vehicle_status_manager:main',
        ],
    },
)
