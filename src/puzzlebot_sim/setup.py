from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'puzzlebot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    
    data_files=[
        # Index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),

        # Config files
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.[yma]*'))),

        # RViz
        (os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*.rviz'))),

        # Meshes
        (os.path.join('share', package_name, 'meshes'),
            glob(os.path.join('meshes', '*.stl'))),

        # URDF
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*.urdf'))),
    ],

    install_requires=['setuptools'],

    zip_safe=True,
    maintainer='Emanuel Vega',
    maintainer_email='A01710366@tec.mx',
    description='Puzzlebot simulation with joint states',
    license='Apache-2.0',

    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
        'console_scripts': [
            'puzzlebot_sim = puzzlebot_sim.puzzlebot_sim:main',

            'joint_state_publisher = puzzlebot_sim.joint_state_publisher:main',
            'localization = puzzlebot_sim.localization:main',
            'controller = puzzlebot_sim.controller:main',
            'path_generator = puzzlebot_sim.path_generator:main',
        ],
    },
)