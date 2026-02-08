import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # パッケージ名の定義
    package_name = 'moebius_ros2'

    # EKFの設定ファイル(ekf.yaml)のパスを取得
    # 注意: setup.pyの data_files で config フォルダがインストールされる設定になっている必要があります
    ekf_config_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'ekf.yaml'
    )

    return LaunchDescription([
        # ---------------------------------------------------------
        # 1. Static Transform Publisher (base_link -> imu_link)
        # ---------------------------------------------------------
        # EKFが計算するために、IMUがロボットのどこにあるかを定義します。
        # ここでは「ロボットの中心(0,0,0)と同じ場所」としています。
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
            output='screen'
        ),

        # ---------------------------------------------------------
        # 2. Extended Kalman Filter (robot_localization)
        # ---------------------------------------------------------
        # ドライバからのオドメトリとIMUを統合して、正確な位置(TF)を出力します。
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path],
        ),

        # ---------------------------------------------------------
        # 3. Moebius Driver (Your Driver)
        # ---------------------------------------------------------
        # ロボットを制御し、センサーデータを送るドライバです。
        # ※ entry_point名が 'driver_node_3' であると仮定しています。
        Node(
            package='moebius_ros2',
            executable='moebius_driver_3', 
            name='moebius_driver',
            output='screen',
        )
    ])
