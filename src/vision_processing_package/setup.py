import os
from setuptools import setup, find_packages

package_name = 'vision_processing_package'

def collect_dir_files(directory):
    files = []
    for root, _, filenames in os.walk(directory):
        for filename in filenames:
            files.append(os.path.join(root, filename))
    return files

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', collect_dir_files('launch')),
        ('share/' + package_name + '/config', collect_dir_files('config')),
        ('share/' + package_name + '/models', collect_dir_files('models')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mmichaud',
    maintainer_email='mmichaud@todo.todo',
    description='Vision processing package with custom messages',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'process_object_vision = vision_processing_package.process_object_vision:main',
            'bbox_to_xyz_service = vision_processing_package.bbox_to_xyz_service:main',
            'object_selection = vision_processing_package.object_selection:main',
            'bbox_to_xyz_service_2D = vision_processing_package.bbox_to_xyz_service_2D:main',
            'gst_cam_node = vision_processing_package.gst_cam_node:main',
        ],
    },
)
