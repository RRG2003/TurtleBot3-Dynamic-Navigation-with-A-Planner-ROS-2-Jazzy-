from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'tb3_dynamic_nav_jazzy'

# Collect all model files (recursively include everything inside models/)
model_files = []
for root, _, files in os.walk(os.path.join(package_name, 'models')):
    for f in files:
        model_files.append(os.path.join(root, f))

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        # ROS2 package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Package manifest
        ('share/' + package_name, ['package.xml']),

        # Config + maps
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join(package_name, 'config', '*'))),

        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join(package_name, 'launch', '*.py'))),

        # RViz config
        (os.path.join('share', package_name, 'rviz'),
            glob(os.path.join(package_name, 'rviz', '*.rviz'))),

        # Model files
        (os.path.join('share', package_name, 'models'), model_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rishabh',
    maintainer_email='rishabh@example.com',
    description='TurtleBot3 dynamic navigation with obstacle spawning for ROS2 Jazzy',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ✅ Correct path to spawn_obstacle.py
            'spawn_obstacle = tb3_dynamic_nav_jazzy.spawn_obstacle:main',
        ],
    },
)

