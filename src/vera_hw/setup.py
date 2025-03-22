import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'vera_hw'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # 👈 this is the missing line
    license='Skana Robotics LTD',
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'sbg_publisher = vera_hw.sbg_publisher:main'
        ],
    },
)
