from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'obstacle_avoider_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Fichiers launch
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
        # Fichiers URDF/XACRO
        (os.path.join('share', package_name, 'urdf'),
         glob('urdf/*.urdf.xacro') + glob('urdf/*.urdf')),
        # Fichiers SDF / mondes
        (os.path.join('share', package_name, 'worlds'),
         glob('worlds/*.sdf')),
        # Fichiers de configuration RViz
        (os.path.join('share', package_name, 'config'),
         glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Etudiant ENSA Fes',
    maintainer_email='etudiant@ensa-fes.ma',
    description='Robot Eviteur d Obstacles - Robotique Intelligente ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'evitement_obstacles_node = obstacle_avoider_pkg.evitement_obstacles_node:main',
            'moniteur_etat_node      = obstacle_avoider_pkg.moniteur_etat_node:main',
        ],
    },
)
