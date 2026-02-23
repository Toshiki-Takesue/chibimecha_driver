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

        # IMU移動平均フィルタ
        self.imu_filter_size = 10
        self.imu_ax_buffer = deque(maxlen=self.imu_filter_size)
        self.imu_ay_buffer = deque(maxlen=self.imu_filter_size)
        self.imu_gz_buffer = deque(maxlen=self.imu_filter_size)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.tf_br = TransformBroadcaster(self)
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)

        self.read_thread_handle = threading.Thread(target=self.read_thread, daemon=True)
        self.read_thread_handle.start()

    def cmd_cb(self, msg):
        # 運動方程式は元のまま維持
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

                    # Protocol: S:e1,e2,e3,e4|I:ax,ay,gz
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

                    if len(encoders) != 4 or len(imu_data) != 3: continue

                    now = self.get_clock().now()
                    dt = (now - self.last_time).nanoseconds / 1e9
                    # if dt <= 0: dt = 0.001
                    if dt < 0.001:
                        continue
                    self.last_time = now

                    # Kinematics (元のまま維持)
                    v = [(d * 2 * math.pi * self.wheel_radius / self.cpr) / dt for d in encoders]

                    # Inverse needed? STM32 returns raw ticks.
                    # Adjust signs here if physical rotation is reversed vs command
                    # v[1] = -v[1]
                    # v[2] = -v[2]

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

        # --- EKF対応のためTF配信を停止（コメントアウト） ---
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
        # -----------------------------------------------

        # OdometryメッセージはEKFの入力として必要なので配信し続ける
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
        
        #covariance
        o.twist.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,  # linear x
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,  # linear y
            0.0, 0.0, 0.05, 0.0, 0.0, 0.0,  # linear z
            0.0, 0.0, 0.0, 0.05, 0.0, 0.0,  # angular x
            0.0, 0.0, 0.0, 0.0, 0.05, 0.0,  # angular y
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05   # angular z
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
        # キャリブレーション処理
        if self.imu_offset is None:
            self.imu_samples.append(data.copy())
            
            if len(self.imu_samples) >= self.imu_calibration_count:
                
                # 中央値を使用（外れ値に強い）
                ax_samples = [s[0] for s in self.imu_samples]
                ay_samples = [s[1] for s in self.imu_samples]
                gz_samples = [s[2] for s in self.imu_samples]
                
                ax_samples.sort()
                ay_samples.sort()
                gz_samples.sort()
                
                mid = self.imu_calibration_count // 2
                avg_ax = ax_samples[mid]
                avg_ay = ay_samples[mid]
                avg_gz = gz_samples[mid]
                
                
                self.imu_offset = [avg_ax, avg_ay, avg_gz]
                self.get_logger().info(f"IMU calibration complete!")
                self.get_logger().info(f"Offset: ax={avg_ax:.1f}, ay={avg_ay:.1f}, gz={avg_gz:.1f}")
            else:
                if len(self.imu_samples) % 40 == 0:
                    self.get_logger().info(f"IMU calibrating... {len(self.imu_samples)}/{self.imu_calibration_count}")
            
            return
        
        # オフセット補正
        data_corrected = [
            data[0] - self.imu_offset[0],
            data[1] - self.imu_offset[1],
            data[2] - self.imu_offset[2]
        ]
        # 移動平均フィルタ（追加）
        self.imu_ax_buffer.append(data_corrected[0])
        self.imu_ay_buffer.append(data_corrected[1])
        self.imu_gz_buffer.append(data_corrected[2])
        
        # バッファが満たされるまで待つ
        if len(self.imu_ax_buffer) < self.imu_filter_size:
            return
        
        # 平均を計算
        ax_filtered = sum(self.imu_ax_buffer) / len(self.imu_ax_buffer)
        ay_filtered = sum(self.imu_ay_buffer) / len(self.imu_ay_buffer)
        gz_filtered = sum(self.imu_gz_buffer) / len(self.imu_gz_buffer)
        imu = Imu()
        imu.header.stamp = now.to_msg()
        imu.header.frame_id = 'imu_link'

        # Raw value to physical value conversion is needed here ideally.
        # Assuming MPU6050 default sensitivity for now (just placeholders)
        # Accel: 16384 LSB/g, Gyro: 16.4 LSB/deg/s

        imu.linear_acceleration.x = data[0] / 16384.0 * 9.80665
        imu.linear_acceleration.y = data[1] / 16384.0 * 9.80665
        # z is missing in STM32 code, ignoring

        imu.angular_velocity.z = (data[2] / 16.4 * (math.pi / 180.0))

        #covariance
        # Angular Velocity Covariance (角速度の信頼度)
        imu.angular_velocity_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        
        # Linear Acceleration Covariance (加速度の信頼度)
        imu.linear_acceleration_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        
        # Orientation Covariance (姿勢の信頼度)
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
