import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
import serial, threading, math, time
from collections import deque

class MoebiusDriver(Node):
    def __init__(self):
        super().__init__('moebius_driver')

        self.declare_parameter('publish_tf', True)
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        try:
            self.ser = serial.Serial('/dev/mecanum_base', 460800, timeout=0.1)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            raise e

        self.wheel_radius = 0.040
        self.cpr = 1320.0
        self.lx_ly = 0.255 + 0.230
        self.pwm_limit = 100

        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.last_time = self.get_clock().now()

        # IMUキャリブレーション用
        self.imu_offset = None
        self.imu_samples = []
        self.imu_calibration_count = 200

        # IMU移動平均フィルタ (6軸すべてに拡張)
        self.imu_filter_size = 10
        self.imu_ax_buffer = deque(maxlen=self.imu_filter_size)
        self.imu_ay_buffer = deque(maxlen=self.imu_filter_size)
        self.imu_az_buffer = deque(maxlen=self.imu_filter_size)
        self.imu_gx_buffer = deque(maxlen=self.imu_filter_size)
        self.imu_gy_buffer = deque(maxlen=self.imu_filter_size)
        self.imu_gz_buffer = deque(maxlen=self.imu_filter_size)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.tf_br = TransformBroadcaster(self)
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)

        self.read_thread_handle = threading.Thread(target=self.read_thread, daemon=True)
        self.read_thread_handle.start()

    def cmd_cb(self, msg):
        vy, vx, wz = msg.linear.x, msg.linear.y, msg.angular.z * 1.5
        f = (self.lx_ly)

        speed_a = -vx + vy + f * wz
        speed_b = vx + vy - f * wz
        speed_c = -vx + vy - f * wz
        speed_d = vx + vy + f * wz

        m1 = speed_a * self.pwm_limit
        m2 = speed_b * self.pwm_limit
        m3 = speed_c * self.pwm_limit
        m4 = speed_d * self.pwm_limit

        cmd_str = f"{int(m1)},{int(m2)},{int(m3)},{int(m4)}\n"
        try:
            self.ser.write(cmd_str.encode())
        except serial.SerialException:
            self.get_logger().warn("Failed to write to serial")

    def read_thread(self):
        while rclpy.ok():
            if self.ser.in_waiting:
                try:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line: continue

                    # Protocol: S:e1,e2,e3,e4|I:ax,ay,az,gx,gy,gz
                    if not line.startswith("S:"): continue

                    parts = line.split('|')
                    if len(parts) < 2: continue

                    enc_str = parts[0].replace("S:", "")
                    imu_str = parts[1].replace("I:", "")

                    try:
                        encoders = [int(x) for x in enc_str.split(',')]
                        imu_data = [int(x) for x in imu_str.split(',')]
                    except ValueError:
                        continue

                    # IMUデータが6つであることを確認
                    if len(encoders) != 4 or len(imu_data) != 6: continue

                    now = self.get_clock().now()
                    dt = (now - self.last_time).nanoseconds / 1e9
                    if dt < 0.001:
                        continue
                    self.last_time = now

                    v = [(d * 2 * math.pi * self.wheel_radius / self.cpr) / dt for d in encoders]

                    v1, v2, v3, v4 = -v[0], v[1], v[2], -v[3]

                    vy = (-v1 + v2 - v3 + v4) / 4.0
                    vx = (v1 + v2 + v3 + v4) / 4.0
                    wz = (v1 - v2 - v3 + v4) / (2.0 * self.lx_ly) * 1.5

                    delta_th = wz * dt
                    delta_x = (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
                    delta_y = (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt

                    self.x += delta_x
                    self.y += delta_y
                    self.th += delta_th

                    self.pub_odom(now, vx, vy, wz)
                    self.pub_imu(now, imu_data)

                except Exception as e:
                    self.get_logger().warn(f"Read error: {e}")

    def pub_odom(self, now, vx, vy, wz):
        q = [0.0, 0.0, math.sin(self.th/2), math.cos(self.th/2)]

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_br.sendTransform(t)

        o = Odometry()
        o.header.stamp = now.to_msg()
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation.z = q[2]
        o.pose.pose.orientation.w = q[3]
        o.twist.twist.linear.x = vx
        o.twist.twist.linear.y = vy
        o.twist.twist.angular.z = wz

        o.twist.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.05, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.05, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.05, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05
        ]

        o.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01
        ]

        self.odom_pub.publish(o)

    def pub_imu(self, now, data):
        # キャリブレーション（オフセット計算）はマイコン側で完了しているため削除！
        # 受信したデータをそのまま移動平均フィルタのバッファに追加します
        self.imu_ax_buffer.append(data[0])
        self.imu_ay_buffer.append(data[1])
        self.imu_az_buffer.append(data[2])
        self.imu_gx_buffer.append(data[3])
        self.imu_gy_buffer.append(data[4])
        self.imu_gz_buffer.append(data[5])

        # バッファが満たされるまで待つ（最初の10回分=ほんの一瞬だけ）
        if len(self.imu_ax_buffer) < self.imu_filter_size:
            return

        # 移動平均を計算してノイズを減らす
        ax_f = sum(self.imu_ax_buffer) / len(self.imu_ax_buffer)
        ay_f = sum(self.imu_ay_buffer) / len(self.imu_ay_buffer)
        az_f = sum(self.imu_az_buffer) / len(self.imu_az_buffer)
        gx_f = sum(self.imu_gx_buffer) / len(self.imu_gx_buffer)
        gy_f = sum(self.imu_gy_buffer) / len(self.imu_gy_buffer)
        gz_f = sum(self.imu_gz_buffer) / len(self.imu_gz_buffer)

        imu = Imu()
        imu.header.stamp = now.to_msg()
        imu.header.frame_id = 'imu_link'

        # 物理量へ変換（1G = 16384 LSB, 1deg/s = 16.4 LSB）
        imu.linear_acceleration.x = ax_f / 16384.0 * 9.80665
        imu.linear_acceleration.y = ay_f / 16384.0 * 9.80665
        imu.linear_acceleration.z = az_f / 16384.0 * 9.80665

        imu.angular_velocity.x = gx_f / 16.4 * (math.pi / 180.0)
        imu.angular_velocity.y = gy_f / 16.4 * (math.pi / 180.0)
        imu.angular_velocity.z = gz_f / 16.4 * (math.pi / 180.0)

        # 共分散行列（EKF用）
        imu.angular_velocity_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]

        imu.linear_acceleration_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]

        imu.orientation_covariance = [
            -1.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0
        ]
        
        self.imu_pub.publish(imu)

def main():
    rclpy.init()
    node = MoebiusDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main() 
