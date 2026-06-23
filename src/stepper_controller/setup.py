import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'stepper_controller'

setup(
    name=package_name,
    version='0.0.1',
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
    description='Stepper motor controller untuk 3D scanner',
    license='MIT',
    entry_points={
        'console_scripts': [
            'stepper_node = stepper_controller.stepper_node:main',
            'mapping_3d = stepper_controller.mapping_3d:main',
        ],
    },
)
