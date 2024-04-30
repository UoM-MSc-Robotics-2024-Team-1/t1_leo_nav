from setuptools import find_packages, setup
import os
from glob import glob

package_name = 't1_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #Add behaviour tree xml files
        (os.path.join('share', package_name, 'behavior_tree_xml'), glob(os.path.join('behavior_tree_xml', '*.xml'))),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*yaml*'))),
        # Include map (.yaml and .pgm) files
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.*yaml*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sfr2023',
    maintainer_email='sfr2023@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'custom_map_saver = t1_nav.custom_map_saver_node:main',
            'explore = t1_nav.explore:main',
            'simple_nav = t1_nav.simple_nav:main',
            'nav2_nav_test = t1_nav.nav2_nav_test:main',
            'robot_navigator = t1_nav.robot_navigator:main',
            'patrol_service_node = t1_nav.patrol_service_node:main',
            'patroling_node = t1_nav.patroling_node:main',
            'robot_state_machine = t1_nav.robot_state_machine:main',
            'main = t1_ui.main:main',
            'drive_to_apriltag = t1_nav.drive_to_apriltag:main',
        ],
    },
)
