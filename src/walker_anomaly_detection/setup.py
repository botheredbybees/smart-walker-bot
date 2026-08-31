from setuptools import find_packages, setup

package_name = 'walker_anomaly_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/anomaly_detection.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='botheredbybees',
    maintainer_email='botheredbybees@gmail.com',
    description='Fall/anomaly detection for smart-walker-bot: ESP32-streamed IMU data, pure free-fall+impact and sustained-tilt detection, publishing alerts.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'anomaly_detection_node = walker_anomaly_detection.anomaly_detection_node:main',
        ],
    },
)
