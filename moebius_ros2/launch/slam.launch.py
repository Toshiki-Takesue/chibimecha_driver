import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # ==========================================
    # ▼ 設定エリア
    # ==========================================
    
    # configファイルのパスを取得
    config_file = os.path.join(
        get_package_share_directory('moebius_ros2'),
        'config',
        'ps4_config.yaml'
    )

    # 1. LiDARの位置 [x, y, z, yaw, pitch, roll]
    lidar_transform = ['0.0', '0.0', '0.27', '0', '0', '0']
    
    # 2. IMUの位置
    imu_transform = ['0.0', '0.0', '0.03', '0', '0', '0']

    # 3. LiDARのIPアドレス
    lidar_ip = '192.168.1.110'
    
    # ==========================================

    return LaunchDescription([
        # --------------------------------------
        # 1. 足回り & オドメトリ (Moebius Driver)
        # --------------------------------------
        Node(
            package='moebius_ros2', 
            executable='moebius_driver',
            name='moebius_driver',
            output='screen',
            parameters=[{'port': '/dev/ttyUSB0'}] 
        ),

        # --------------------------------------
        # 2. PS4コントローラ関連 (ここが追加部分)
        # --------------------------------------
        # (A) joy_node: コントローラの信号を読む
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'dev': '/dev/input/js0', 'deadzone': 0.05, 'autorepeat_rate': 20.0}]
        ),
        # (B) teleop_twist_joy: 信号を速度指令(cmd_vel)に変換
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[config_file],
            # ROS 2の仕様に合わせてリマップ
            remappings=[('/cmd_vel', '/cmd_vel')]
        ),

        # --------------------------------------
        # 3. LiDARドライバ
        # --------------------------------------
        Node(
            package='urg_node',
            executable='urg_node_driver',
            name='urg_node',
            output='screen',
            parameters=[{
                'ip_address': lidar_ip,
                'laser_frame_id': 'laser',
                'angle_min': -2.356, 
                'angle_max': 2.356,
                'calibrate_time': True
            }]
        ),

        # --------------------------------------
        # 4. TF (位置関係)
        # --------------------------------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_lidar',
            arguments=lidar_transform + ['base_footprint', 'laser']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_imu',
            arguments=imu_transform + ['base_footprint', 'imu_link']
        ),

        # --------------------------------------
        # 5. SLAM Toolbox (地図作成)
        # --------------------------------------
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'base_frame': 'base_footprint',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'mode': 'mapping',
                'transform_timeout': 1.0,
                'tf_buffer_duration': 30.0
            }]
        )
    ])
