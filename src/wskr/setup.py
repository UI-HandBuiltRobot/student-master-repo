import os
from setuptools import find_packages, setup

package_name = 'wskr'

def collect_model_data(package_models_dir='wskr/models'):
    entries = {}
    for root, _, files in os.walk(package_models_dir):
        json_files = [os.path.join(root, f) for f in files if f.endswith('.json')]
        if not json_files:
            continue
        # compute the relative subpath inside models ('' for top-level)
        rel_dir = os.path.relpath(root, package_models_dir)
        if rel_dir == '.':
            target_dir = f'share/{package_name}/models'
        else:
            target_dir = f'share/{package_name}/models/{rel_dir}'
        entries.setdefault(target_dir, []).extend(json_files)
    # return list of (target_dir, [files...]) tuples suitable for setup(data_files=...)
    return list(entries.items())

models_data_files = collect_model_data()
# then pass models_data_files into data_files along with other entries, e.g.:
# data_files=[ ..., *models_data_files ]

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    # base data_files entries
    data_files=(
        [
            ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
            ('share/' + package_name + '/config', ['config/Whisker_Calibration.json']),
            ('share/' + package_name + '/launch', ['launch/wskr.launch.py']),
        ]
        # append the per-directory model entries collected by collect_model_data()
        + models_data_files
    ),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Project5 User',
    maintainer_email='user@example.com',
    description='Floor masking and whisker range estimation nodes.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'wskr_floor = wskr.wskr_floor_node:main',
            'wskr_range = wskr.wskr_range_node:main',
            'wskr_approach_action = wskr.approach_action_server:main',
            'wskr_dead_reckoning = wskr.dead_reckoning_node:main',
            'wskr_autopilot = wskr.wskr_autopilot:main',
        ],
    },
)
