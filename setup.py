from pathlib import Path
from setuptools import setup

package_name = 'rtcosmik_ros'


def package_files(directory: str):
    """Return data_files tuples preserving subfolders under share/<package>/<directory>."""
    root = Path(directory)
    if not root.exists():
        return []

    data = []
    for file_path in root.rglob('*'):
        if file_path.is_file():
            install_dir = f'share/{package_name}/{file_path.parent.as_posix()}'
            data.append((install_dir, [str(file_path)]))
    return data


data_files = [
    ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
    (f'share/{package_name}', ['package.xml']),
]

# Standard ROS resources + optional robot assets (if present).
data_files += package_files('launch')
data_files += package_files('rviz')
data_files += package_files('urdf')
data_files += package_files('meshes')
data_files += package_files('config')

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Guilhem Saurel',
    maintainer_email='gsaurel@laas.fr',
    description='ROS 2 bridge for RT-COSMIK',
    license='BSD-2-Clause',
    entry_points={
        'console_scripts': [
            'marker_bridge = rtcosmik_ros.marker_bridge_node:main',
        ],
    },
)
