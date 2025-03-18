from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vera_sim'

setup(
    name=package_name,
    version='0.0.1',  # Ensure version is set
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    license='Copyright (c) 2024 Skana Robotics LTD',
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'simple_boat_sim = vera_sim.simple_boat_sim:main',
        ],
    }
)
