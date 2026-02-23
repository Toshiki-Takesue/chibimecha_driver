import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import launch_ros.actions
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory, get_package_share_path

def generate_launch_description():
    pkg_name = 'moebius_ros2'
    description_pkg_path = get_package_share_path('mecanumrover_description')
    
    # JOY設定ファイルのパス
    joy_config = os.path.join(get_package_share_directory(pkg_name), 'config', 'ps4.config.yaml')
    default_model_path = description_pkg_path / 'urdf/mecanum3.xacro'

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=str(default_model_path),
        description='Path to robot urdf file'
    )

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    # 1. ロボットモデル配信 (TF_Static用)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    # 2. 足回りドライバ (ここでTFを出すように設定！)
    driver_node = Node(
        package='moebius_ros2',
        executable='moebius_driver_3',
        name='moebius_driver',
        output='screen',
        parameters=[{
            'publish_tf': False,  # 重要: これで生のTFが出ます
            'pwm_limit': 150,
        }]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory("moebius_ros2"), 'config', 'ekf.yaml')]
    )

    # 3. LIDAR
    lidar_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        parameters=[{
            'ip_address': '192.168.1.110',
            'laser_frame_id': 'laser',
            'angle_min': -2.356, 
            'angle_max': 2.356
        }]
    )

    # 4. コントローラ
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'device_name': 'Wireless Controller', 'deadzone': 0.05}]
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joy_config]
    )

    # 5. TF (LIDARの位置)
    tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.3', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        driver_node,
        lidar_node,
        joy_node,
        teleop_node,
        tf_laser,
        ekf_node
    ])
