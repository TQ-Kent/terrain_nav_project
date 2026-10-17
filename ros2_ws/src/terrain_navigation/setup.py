from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'terrain_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Nav2 config files
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        # Maze layouts for the headless simulator
        (os.path.join('share', package_name, 'maps'),
            glob(os.path.join('maps', '*.txt'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ken',
    maintainer_email='thanhquan.kent@gmail.com',
    description='Nav2 configuration, frontier exploration and headless simulator for the maze test',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'set_algo = terrain_navigation.set_algo:main',
            'goal_explorer = terrain_navigation.goal_explorer:main',
            'headless_sim = terrain_navigation.headless_sim:main',
        ],
    },
)
