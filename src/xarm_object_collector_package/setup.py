from setuptools import setup, find_packages

package_name = 'xarm_object_collector_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/xarm_object_collector_ga.launch.py']),
        ('share/' + package_name + '/data',
            ['data/q_table.csv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mmichaud',
    maintainer_email='megan-michaud@uiowa.edu',
    description='Object collection behavior for xARM robot.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'object_collector_action_server = xarm_object_collector_package.object_collector_action_server:main',
            'xarm_hardware_node = xarm_object_collector_package.xarm_hardware_node:main',
            'q_learning_hand = xarm_object_collector_package.q_learning_hand:main',
        ],
    },
)
