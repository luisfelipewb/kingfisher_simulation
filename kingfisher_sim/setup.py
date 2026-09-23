import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'kingfisher_sim'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luis Batista',
    maintainer_email='luisfelipewb@gmail.com',
    description='Simulation glue for the Clearpath Kingfisher in VRX.',
    license='BSD',
    entry_points={
        'console_scripts': [
            'cmd_drive_translate = kingfisher_sim.cmd_drive_translate:main',
        ],
    },
)
