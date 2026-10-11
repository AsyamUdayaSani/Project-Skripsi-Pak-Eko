import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sweep_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raspitampan',
    maintainer_email='raspitampan@todo.todo',
    description='Mapping 3D dengan jumlah sweep yang bisa diatur lewat parameter',
    license='MIT',
    entry_points={
        'console_scripts': [
            'stepper_sweep_node = sweep_mapping.stepper_sweep_node:main',
            'mapping_3d_sweep = sweep_mapping.mapping_3d_sweep:main',
            'bag_recorder_node = sweep_mapping.bag_recorder_node:main',
        ],
    },
)
