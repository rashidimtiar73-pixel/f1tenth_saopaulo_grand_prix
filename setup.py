from setuptools import setup
import os
from glob import glob

package_name = 'f1tenth_saopaulo_grand_prix'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name, f'{package_name}.controllers'],
    package_dir={'': '.', f'{package_name}.controllers': 'controllers'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Prantor Imtiar Rashid',
    maintainer_email='rashidimtiar73-pixel@users.noreply.github.com',
    description='F1Tenth Reactive Autonomous Racing Package for Sao Paulo Track',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamic_apex_gap_node = controllers.dynamic_apex_gap_node:main',
        ],
    },
)
