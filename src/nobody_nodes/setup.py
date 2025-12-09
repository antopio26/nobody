from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import setup


package_name = 'nobody_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('nobody_nodes', 'launch', '*launch.*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('nobody_nodes', 'config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "pointcloud_filter_node = nobody_nodes.pointcloud_filter_node_torch:main",
            "target_pos_estimation = nobody_nodes.target_pos_estimation:main",
        ],
    },
)
