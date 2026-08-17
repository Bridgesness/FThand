import os
from glob import glob
from setuptools import setup

package_name = 'orcahand_teleop_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 把 launch 目录装进去，ros2 launch 才能找到
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='haoyuanwu',
    maintainer_email='haoyuanwu@example.com',
    description='OrcaHand teleop ROS2 nodes (glove/retargeter/hand)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_glove = orcahand_teleop_ros2.fake_glove_publisher:main',
            'glove_driver = orcahand_teleop_ros2.glove_driver_node:main',
            'retargeter = orcahand_teleop_ros2.retargeter_node:main',
            'hand_controller = orcahand_teleop_ros2.hand_controller_node:main',
            'record_targets = orcahand_teleop_ros2.record_targets:main',
            'replay_targets = orcahand_teleop_ros2.replay_targets:main',
            'record_combined = orcahand_teleop_ros2.record_combined:main',
            'replay_combined = orcahand_teleop_ros2.replay_combined:main',
            'replay_sync = orcahand_teleop_ros2.replay_sync:main',
        ],
    },
)
