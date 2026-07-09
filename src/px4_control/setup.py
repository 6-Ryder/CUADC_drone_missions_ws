import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'px4_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryder',
    maintainer_email='ryder@todo.todo',
    description='PX4 flight control abstraction layer',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'px4_bridge = px4_control.px4_bridge:main',
        ],
    },
)
