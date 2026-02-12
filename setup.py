from setuptools import setup
from glob import glob

package_name = 'rtcosmik_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (f'share/{package_name}/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LAAS-CNRS',
    maintainer_email='gsaurel@laas.fr',
    description='ROS 2 bridge for RT-COSMIK',
    license='BSD-2-Clause',
    entry_points={
        'console_scripts': [
            'marker_bridge = rtcosmik_ros.marker_bridge_node:main',
        ],
    },
)
