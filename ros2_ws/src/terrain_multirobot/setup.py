from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'terrain_multirobot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ken',
    maintainer_email='thanhquan.kent@gmail.com',
    description='CBBA task allocation for several Nav2 robots',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'cbba_agent = terrain_multirobot.cbba_agent_node:main',
            'task_publisher = terrain_multirobot.task_publisher_node:main',
        ],
    },
)
