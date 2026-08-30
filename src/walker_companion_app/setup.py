from setuptools import find_packages, setup

package_name = 'walker_companion_app'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/dashboard_app.launch.py']),
        ('share/' + package_name + '/web', ['web/index.html']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='botheredbybees',
    maintainer_email='botheredbybees@gmail.com',
    description='Local-network web dashboard for smart-walker-bot: robot pose, Nav2 status, live map, and the walker_llm_bridge conversation log.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard_app_node = walker_companion_app.dashboard_app_node:main',
        ],
    },
)
