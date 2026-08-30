from setuptools import find_packages, setup

package_name = 'walker_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/walker_nav.launch.py',
            'launch/nav2.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/slam_toolbox_params.yaml',
            'config/nav2_params.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='botheredbybees',
    maintainer_email='botheredbybees@gmail.com',
    description='SLAM + Nav2 integration layer for smart-walker-bot: a simulated LiDAR feeding slam_toolbox, and nav2_bringup\'s navigation stack configured against the live map, until real hardware exists.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_lidar_node = walker_nav.fake_lidar_node:main',
        ],
    },
)
