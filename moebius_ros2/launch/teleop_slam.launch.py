import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory, get_package_share_path

def generate_launch_description():
    # --- 1. パスと設定の取得 ---
    pkg_name = 'moebius_ros2'
    bringup_pkg_path = get_package_share_path('mecanumrover3_bringup')
    description_pkg_path = get_package_share_path('mecanumrover_description')

    # 設定ファイルのパス
    joy_config = os.path.join(get_package_share_directory(pkg_name), 'config', 'ps4.config.yaml')
    ekf_config = os.path.join(get_package_share_directory(pkg_name), 'config', 'ekf.yaml')
    slam_config = os.path.join(get_package_share_directory(pkg_name), 'config', 'mapper_params_online_async.yaml')
    
    # モデルとRviz設定
    default_model_path = description_pkg_path / 'urdf/mecanum3.xacro'
    # SLAM用のRviz設定があればそれを指定、なければデフォルト
    default_rviz_config_path = bringup_pkg_path / 'rviz/mecanum3.rviz' 

    # --- 2. Launch引数 ---
    model_arg = DeclareLaunchArgument(
        name='model', 
        default_value=str(default_model_path),
        description='Absolute path to robot urdf file'
    )

    # --- 3. ロボット記述 (URDF) ---
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    # --- 4. ハードウェア & センサ ---
    
    # A. 足回りドライバ (Encoders + IMU)
    driver_node = Node(
        package='moebius_ros2',
        executable='moebius_driver_3',
        name='moebius_driver',
        output='screen',
        parameters=[{
            'port': '/dev/ttyUSB0',
            'baud': 460800,
            'pwm_limit': 150, # 速度リミットはお好みで
        }]
    )

    # B. LIDAR (Hokuyo URG)
    lidar_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        parameters=[{
            'ip_address': '192.168.1.110',
            'laser_frame_id': 'laser',
            'angle_min': -2.356, # -135度
            'angle_max': 2.356   # +135度
        }]
    )

    # C. コントローラ (Joy + Teleop)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_name': 'Wireless Controller',
            'deadzone': 0.05
        }]
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joy_config]
    )

    # --- 5. 座標変換 (TF) ---

    # base_link -> laser (実機に合わせて調整: 前0.1m, 上0.1mと仮定)
    tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.1', '0.0', '0.1', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )

    # base_link -> imu_link
    tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'imu_link']
    )

    # --- 6. 推定・地図作成 (EKF + SLAM) ---

    # EKF (Odom + IMU -> odom座標系)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config]
    )

    # SLAM Toolbox (Scan + Odom -> map座標系)
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
       output='screen',
        parameters=[slam_config]
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
        tf_imu,
        ekf_node,
        slam_node
    ]) 
