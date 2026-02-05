import os
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # --- 1. パスの取得 ---
    moebius_pkg = get_package_share_directory('moebius_ros2')
    description_pkg_path = get_package_share_path('mecanumrover_description')
    bringup_pkg_path = get_package_share_path('mecanumrover3_bringup')
    
    joy_config = os.path.join(moebius_pkg, 'config', 'ps4.config.yaml')
    default_model_path = description_pkg_path / 'urdf/mecanum3.xacro'
    default_rviz_config_path = bringup_pkg_path / 'rviz/mecanum3.rviz'

    # --- 2. Launch引数の設定 ---
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig', 
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file'
    )

    model_arg = DeclareLaunchArgument(
        name='model', 
        default_value=str(default_model_path),
        description='Absolute path to robot urdf file'
    )

    # --- 3. ロボット記述 (URDF/Xacro) の読み込み ---
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    # --- 4. ノード定義 ---
    
    # A. 状態監視・可視化系
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # B. STM32ドライバ (最新の driver_node_3 を使用)
    moebius_driver_node = Node(
        package='moebius_ros2',
        executable='moebius_driver_3',  # ← ここを moebius_driver_3 に修正
        name='moebius_driver',
        output='screen',
        parameters=[{
            'port': '/dev/ttyUSB0', 
            'baud': 460800,
            'pwm_limit': 150,
            'tread': 0.255,      # 左右の車輪間隔
            'wheelbase': 0.230   # 前後の車輪間隔
        }]
    )

    # C. コントローラ入力 (Joy)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_name': 'Wireless Controller',
            'deadzone': 0.05
        }]
    )

    # D. 速度変換 (Teleop)
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joy_config]
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        moebius_driver_node,
        joy_node,
        teleop_node
    ])
