from setuptools import find_packages, setup

package_name = 'avp_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/multi_avp_launch.py']),
        ('share/' + package_name + '/scripts', ['scripts/echo_avp_topics.sh']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zubair Islam',
    maintainer_email='zubxxr@gmail.com',
    description='Top-level orchestrator node for AVP system',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'avp_node = avp_node.avp_node:main',
        ],
    },
)

