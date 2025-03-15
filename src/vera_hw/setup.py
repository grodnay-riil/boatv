import os
from glob import glob
from setuptools import setup

package_name = 'vera_hw'

setup(
    name=package_name,
    license='Skana Robotics LTD',
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
         ('share/' + package_name, ['package.xml']),  # Ensure package.xml is installed
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),  # Install marker
    ]
)