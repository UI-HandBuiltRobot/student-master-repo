from setuptools import setup, find_packages
from glob import glob
import os
import sys

package_name = 'system_manager_package'

# Add src directory to path so we can import scripts directly
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),

    data_files=[
        # ROS 2 package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],

    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='Project5 User',
    maintainer_email='user@example.com',
    description='System manager package for robot coordination.',
    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'state_manager = system_manager_package.state_manager:main',
            'search_supervisor = system_manager_package.search_supervisor:main',
        ],
    },
)