from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'f450_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dave',
    maintainer_email='dave@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub_velocity = f450_description.pub_velocity:main',
            'function = f450_description.function:main',
            'sub_imu = f450_description.sub_imu:main',
            'pubsub = f450_description.pubsub:main',
            'pruebas  = f450_description.pruebas:main',
            'var_2_vai  = f450_description.var_2_vai:main',
            'prueba_p1  = f450_description.prueba_p1:main',
            'prueba_p1g  = f450_description.prueba_p1g:main',
            'prueba_general  = f450_description.prueba_general:main',
        ],
    },
)