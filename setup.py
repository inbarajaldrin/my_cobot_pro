from setuptools import find_packages, setup
import os
import glob

package_name = 'my_cobot_pro'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Install Launch Files
        ('share/' + package_name + '/launch', 
            glob.glob(os.path.join('launch', '*.py'))),

        # Install URDF Files
        ('share/' + package_name + '/urdf', 
            glob.glob(os.path.join('urdf', '*.urdf'))),

        # Install Meshes
        ('share/' + package_name + '/meshes',
            glob.glob(os.path.join('meshes', '*.dae'))),

        # Install Config Files
        ('share/' + package_name + '/config', 
            glob.glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='aldrininbaraj@gmail.com',
    description='ROS 2 package for MyCobot Pro',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
