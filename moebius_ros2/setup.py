import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'moebius_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launchフォルダの中身をインストール先にコピーする指示
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # configフォルダの中身をコピーする指示 (globを使って.yaml全部を対象に変更)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'moebius_driver = moebius_ros2.driver_node:main',
        ],
    },
)