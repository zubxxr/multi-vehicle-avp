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
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ovin',
    maintainer_email='ovin@todo.todo',
    description='AVP Manager Nodes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drop_off_zone_queue_manager = avp_managers.drop_off_zone_queue_manager:main',
            'parking_spot_reservation_manager = avp_managers.parking_spot_reservation_manager:main',
            'vehicle_count_manager = avp_managers.vehicle_count_manager:main',
        ],
    },
)

