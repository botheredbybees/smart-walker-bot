from setuptools import find_packages, setup

package_name = 'walker_gait_metrics'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gait_metrics.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='botheredbybees',
    maintainer_email='botheredbybees@gmail.com',
    description=(
        "Wellness gait metrics (step count, step length) for smart-walker-bot, derived from "
        "walker_anomaly_detection's IMU stream and walker_motor_driver's odometry."
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gait_metrics_node = walker_gait_metrics.gait_metrics_node:main',
        ],
    },
)
