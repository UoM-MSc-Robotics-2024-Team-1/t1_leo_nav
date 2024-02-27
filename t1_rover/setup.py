from setuptools import find_packages, setup
import os
from glob import glob

package_name = 't1_rover'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include URDF (.urdf) files
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf.xacro*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro*'))),
        # Include rviz (.rviz) files
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        # Include world (.sdf or dae) files
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '**/**.[sd][da][fe]'), recursive=True)),
        # Include config (.yaml) files
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*yaml*'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.*yaml*'))),
        # Include map (.yaml and .pgm) files
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.[yp][ag][m]'))),
        # Include Meshes
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.[sS][tT][lL]'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.[dD][aA][eE]'))),
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
            'custom_map_saver = t1_rover.custom_map_saver_node:main',
        ],
    },
)
