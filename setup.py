from setuptools import setup
import os
from glob import glob

package_name = 'pmr_tp_final'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Theo Cruz, Nikolas Barbosa',
    maintainer_email='theofonsecacruz@gmail.com, nikolasbarbosaextras@gmail.com',
    description='Implementation of collision avoidance with VO CBFs with slack and safety CBFs',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = pmr_tp_final.controller:main',
        ],
    },
)
