from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cuzk_tools'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch')+glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        # *[(os.path.join('share', package_name, os.path.dirname(yaml_file)), [yaml_file]) for yaml_file in glob('config/**/*.yaml', recursive=True)],
        # # add rviz files
        # *[(os.path.join('share', package_name, os.path.dirname(rviz_file)), [rviz_file]) for rviz_file in glob('rviz/**/*.rviz', recursive=True)],
        # (os.path.join('share', package_name), ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='herolada',
    maintainer_email='herolada@fel.cvut.cz',
    description='Package for working with elevation and topography data from CUZK (ČÚZK---Český úřad zeměměřický a katastrální).',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'elevation_odom = cuzk_tools.elevation_odom:main',
        ],
    },
        
        
)
