import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'depth_anything_v2_ros2'
submodule_name = 'depth_anything_v2'
metric_name = 'metric_depth'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Install Python submodule files Default DepthAnything V2 
        (os.path.join('lib', package_name, submodule_name, submodule_name),
         glob(os.path.join(submodule_name, submodule_name, '*.py'))),
        (os.path.join('lib', package_name, submodule_name, submodule_name, 'dinov2_layers'),
         glob(os.path.join(submodule_name, submodule_name, 'dinov2_layers', '*.py'))),
        (os.path.join('lib', package_name, submodule_name, submodule_name, 'util'),
         glob(os.path.join(submodule_name, submodule_name, 'util', '*.py'))),
        # Install Python submodule files Metric DepthAnything V2 
        (os.path.join('lib', package_name, submodule_name, metric_name, submodule_name),
         glob(os.path.join(submodule_name, metric_name, submodule_name, '*.py'))),
        (os.path.join('lib', package_name, submodule_name, metric_name, submodule_name, 'dinov2_layers'),
         glob(os.path.join(submodule_name, metric_name, submodule_name, 'dinov2_layers', '*.py'))),
        (os.path.join('lib', package_name, submodule_name, metric_name, submodule_name, 'util'),
         glob(os.path.join(submodule_name, metric_name, submodule_name, 'util', '*.py'))), 
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.pth'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alberto Tudela',
    maintainer_email='ajtudela@gmail.com',
    author='Óscar Pons Fernández',
    author_email='oscarpf22@gmail.com',
    description='ROS2 Wrapper for DepthAnything V2 model',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_anything_v2_ros2 = depth_anything_v2_ros2.depth_anything_v2_ros2:main'
        ],
    },
)
