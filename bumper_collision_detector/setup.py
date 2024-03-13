from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bumper_collision_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lolddohaja',
    maintainer_email='progryu@zetabank.co.kr',
    description='bumper monitoring package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bumper_monitor_node = bumper_collision_detector.bumper_monitor_node:main'
        ],
    },
)
