from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vera_description'

def get_all_model_files():
    data = []
    for root, _, files in os.walk('models'):
        if files:
            # Install path under share/<package_name>/models/...
            rel_path = os.path.relpath(root, 'models')
            install_path = os.path.join('share', package_name, 'models', rel_path)
            full_paths = [os.path.join(root, f) for f in files]
            data.append((install_path, full_paths))
    return data

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*'))),
    ] + get_all_model_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='verauser',
    maintainer_email='grodnay@riil.tech',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
