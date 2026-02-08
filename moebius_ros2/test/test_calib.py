import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
import time

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

class CalibrationTest(Node):
    def __init__(self):
        super().__init__('calibration_test')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        # 状態管理
        self.state = 0
        self.start_x = None
        self.start_y = None
        self.start_yaw = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # スタック検知用
        self.last_pos_update_time = time.time()
        self.last_dist = -1.0

        self.imu_yaw = 0.0
        self.last_imu_time = None

        self.get_logger().info("=== キャリブレーションテスト開始 ===")
        self.get_logger().info("Odomデータ待機中...")

    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        ox = msg.pose.pose.orientation.x
        oy = msg.pose.pose.orientation.y
        oz = msg.pose.pose.orientation.z
        ow = msg.pose.pose.orientation.w
        _, _, self.current_yaw = euler_from_quaternion(ox, oy, oz, ow)

        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.start_yaw = self.current_yaw
            self.get_logger().info("初期位置取得完了。テストを開始します。")
            self.state = 1
            self.last_pos_update_time = time.time()

    def imu_cb(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9
        if self.last_imu_time is not None:
            dt = current_time - self.last_imu_time
            self.imu_yaw += msg.angular_velocity.z * dt
        self.last_imu_time = current_time

    def control_loop(self):
        if self.start_x is None: return

        cmd = Twist()

        # --- フェーズ1: 直進 ---
        if self.state == 1:
            dx = self.current_x - self.start_x
            dy = self.current_y - self.start_y
            dist = math.sqrt(dx*dx + dy*dy)

            # スタック検知（進捗がなければ警告）
            if abs(dist - self.last_dist) > 0.001:
                self.last_dist = dist
                self.last_pos_update_time = time.time()
            elif time.time() - self.last_pos_update_time > 5.0:
                 # 5秒間オドメトリの変化なし
                 pass 

            if dist < 1.0:
                cmd.linear.x = 0.4  # 少し速度を上げる
                # 表示の頻度を下げる
                if int(time.time() * 10) % 5 == 0:
                    print(f"\r直進中: {dist:.3f} / 1.000 m (x:{self.current_x:.2f}, y:{self.current_y:.2f})", end="")
            else:
                cmd.linear.x = 0.0
                self.state = 2
                self.phase_start_time = time.time()
                print(f"\n1.0m 到達判定！ (IMU計測値: {dist:.3f}m)")
                print("2秒停止します...")

        # --- フェーズ2: 停止 ---
        elif self.state == 2:
            cmd.linear.x = 0.0
            if time.time() - self.phase_start_time > 2.0:
                self.state = 3
                self.start_yaw = self.current_yaw
                self.imu_yaw = 0.0
                print("90度回転を開始します...")

        # --- フェズ3: 回転 ---
        elif self.state == 3:
            diff = self.current_yaw - self.start_yaw
            while diff > math.pi: diff -= 2*math.pi
            while diff < -math.pi: diff += 2*math.pi

            target = math.pi / 2.0

            if abs(diff) < target:
                cmd.angular.z = 0.5
                deg = math.degrees(abs(diff))
                if int(time.time() * 10) % 5 == 0:
                    print(f"\r回転中: {deg:.1f} / 90.0 度", end="")
            else:
                cmd.angular.z = 0.0
                self.state = 4
                print(f"\n90度 到達判定！")
                print(f"Odom判定角度: {math.degrees(abs(diff)):.2f}度")
                print("テスト終了。")

        elif self.state == 4:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = CalibrationTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # ここでの二重呼び出し防止
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
