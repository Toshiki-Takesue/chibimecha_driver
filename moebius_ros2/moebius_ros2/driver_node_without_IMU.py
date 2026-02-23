import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial, threading, math

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

        # オドメトリ用のパブリッシャーのみ残す
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
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

                    # Protocol: S:e1,e2,e3,e4|I:ax,ay,gz
                    if not line.startswith("S:"): continue

                    parts = line.split('|')
                    enc_str = parts[0].replace("S:", "")
                    
                    # STM32からIMUデータが送られてきてもここでは無視する
                    try:
                        encoders = [int(x) for x in enc_str.split(',')]
                    except ValueError:
                        continue

                    if len(encoders) != 4: continue

                    now = self.get_clock().now()
                    dt = (now - self.last_time).nanoseconds / 1e9
                    
                    if dt < 0.001:
                        continue
                    self.last_time = now

                    # 順運動学（Kinematics）
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

                    # オドメトリのみ配信
                    self.pub_odom(now, vx, vy, wz)

                except Exception as e:
                    self.get_logger().warn(f"Read error: {e}")

    def pub_odom(self, now, vx, vy, wz):
        q = [0.0, 0.0, math.sin(self.th/2), math.cos(self.th/2)]

        # TF配信（publish_tfがTrueの場合のみ）
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
        
        # Twistの共分散（速度の信頼度）
        o.twist.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,  # x (前進)
            0.0, 0.2, 0.0, 0.0, 0.0, 0.0,   # y (横移動: スリップしやすいので悪化)
            0.0, 0.0, 0.05, 0.0, 0.0, 0.0,  # z
            0.0, 0.0, 0.0, 0.05, 0.0, 0.0,  # roll
            0.0, 0.0, 0.0, 0.0, 0.05, 0.0,  # pitch
            0.0, 0.0, 0.0, 0.0, 0.0, 0.5    # yaw (旋回: IMUなしでズレやすいので悪化)
        ]

        # Poseの共分散（位置の信頼度）
        o.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,  # x
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,  # y (悪化)
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,  # z
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,  # roll
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,  # pitch
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1    # yaw (悪化)
        ]

        self.odom_pub.publish(o)

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