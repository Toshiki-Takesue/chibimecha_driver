import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
import serial
import time
import threading
import math

# --- クォータニオン変換関数 (外部ライブラリ依存を避けるため簡易実装) ---
def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

# --- STM32通信クラス ---
class STM32Interface:
    def __init__(self, port, baud=460800):
        self.port = port
        self.baud = baud
        self.ser = None

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            self.ser.rts = False
            self.ser.dtr = False
            time.sleep(0.1)
            self.ser.dtr = True
            time.sleep(0.1)
            self.ser.dtr = False
            time.sleep(2.0)
            self.ser.reset_input_buffer()
            return True
        except Exception as e:
            return False, e

    def read_line(self):
        if self.ser and self.ser.is_open and self.ser.in_waiting > 0:
            try:
                # エラー文字を無視してデコード
                return self.ser.readline().decode('utf-8', errors='ignore').strip()
            except:
                return None
        return None

    def send_speeds(self, m1, m2, m3, m4):
        if self.ser and self.ser.is_open:
            # Arduino側はパルスやPWMではなく、このドライバからはまだPWM相当の値を送っている前提
            # (cmd_velのロジックは変更しない指示のため)
            cmd = f"{int(m1)},{int(m2)},{int(m3)},{int(m4)}\n"
            try:
                self.ser.write(cmd.encode('utf-8'))
            except Exception:
                pass

    def close(self):
        if self.ser:
            self.ser.close()

# --- ROS 2 ドライバーノード ---
class MoebiusDriver(Node):
    def __init__(self):
        super().__init__('moebius_driver')
        
        # パラメータ設定 
        self.declare_parameter('pwm_limit', 150)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 460800)

        self.pwm_limit = self.get_parameter('pwm_limit').value
        port_name = self.get_parameter('port').value
        baud_rate = self.get_parameter('baud').value

        self.declare_parameter('tread', 0.255)
        self.declare_parameter('wheelbase', 0.230)
        tread = self.get_parameter('tread').value
        wheelbase = self.get_parameter('wheelbase').value
        
        # メカナムの幾何学係数 (Lx + Ly)
        self.geo_factor = (tread + wheelbase) / 2.0 

        # --- オドメトリ計算用変数 ---
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        # パブリッシャ
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # STM32接続
        self.get_logger().info(f"Connecting to {port_name} at {baud_rate}...")
        self.stm = STM32Interface(port_name, baud_rate)
        
        success = self.stm.connect()
        if success is True:
            self.get_logger().info("Connected to STM32 successfully!")
        else:
            self.get_logger().error(f"Connection Failed: {success[1]}")

        # サブスクライバ (cmd_vel) -> 変更なし
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.listener_callback, 10)

        # 受信スレッド開始
        self.running = True
        self.read_thread = threading.Thread(target=self.read_loop)
        self.read_thread.start()
        
        self.get_logger().info(f"Moebius Driver Started (Odom + IMU).")

    # 別スレッドで常にシリアルを監視する
    def read_loop(self):
        while self.running and rclpy.ok():
            line = self.stm.read_line()
            if line:
                # データ形式: "SPD:m1=...,m2=... | IMU:AcX=..."
                self.parse_serial_data(line)
            time.sleep(0.001)

    # 文字列を解析してROSメッセージにする
    def parse_serial_data(self, line):
        try:
            # パイプ | で分割 (SPD部とIMU部)
            sections = line.split('|')
            data = {}

            for section in sections:
                section = section.strip()
                # コロン : でプレフィックスを除去 (SPD: や IMU:)
                if ':' in section:
                    _, content = section.split(':', 1)
                    # カンマ , で各値に分割
                    items = content.split(',')
                    for item in items:
                        if '=' in item:
                            k, v = item.split('=')
                            data[k.strip()] = float(v)

            current_time = self.get_clock().now()

            # --- 1. オドメトリ計算 (SPDデータがある場合) ---
            if 'm1' in data and 'm2' in data and 'm3' in data and 'm4' in data:
                # Arduinoから送られてくるのは m/s 単位の速度
                v1 = data['m1'] # FL
                v2 = data['m2']* -1.0 # FR
                v3 = data['m3']* -1.0 # RL
                v4 = data['m4'] # RR

                self.get_logger().info(f"Commanded Speeds -> m1:{m1:.1f}, m2:{m2:.1f}, m3:{m3:.1f}, m4:{m4:.1f}")
                # メカナム逆運動学 (X型配置)
                # 車体速度 vx, vy, wz を計算
                vx = (v1 + v2 + v3 + v4) / 4.0
                vy = (-v1 + v2 + v3 - v4) / 4.0
                wz = (-v1 + v2 - v3 + v4) / (4.0 * self.geo_factor)

                # 時間経過 (dt) の計算
                dt = (current_time - self.last_time).nanoseconds / 1e9
                self.last_time = current_time

                # 位置の積分 (オドメトリ)
                delta_x = (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
                delta_y = (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt
                delta_th = wz * dt

                self.x += delta_x
                self.y += delta_y
                self.th += delta_th

                # オドメトリメッセージの作成と送信
                self.publish_odometry(current_time, vx, vy, wz)

            # --- 2. IMUデータ送信 ---
            if 'GyZ' in data:
                imu_msg = Imu()
                imu_msg.header.stamp = current_time.to_msg()
                imu_msg.header.frame_id = "imu_link"
                
                # MPU6050の生データを物理量に変換
                # Gyro: /131.0 (deg/s -> rad/s変換が必要だが、ここではスケールのみ。必要なら * 3.14/180)
                # Accel: /16384.0 * 9.8 (G -> m/s^2)
                imu_msg.angular_velocity.z = (data['GyZ'] / 131.0) * (math.pi / 180.0) 
                if 'AcX' in data: imu_msg.linear_acceleration.x = data['AcX'] / 16384.0 * 9.8
                if 'AcY' in data: imu_msg.linear_acceleration.y = data['AcY'] / 16384.0 * 9.8
                
                self.imu_pub.publish(imu_msg)

        except Exception as e:
            # パースエラーなどは無視
            pass

    def publish_odometry(self, current_time, vx, vy, wz):
        # クォータニオンへ変換
        q = euler_to_quaternion(0, 0, self.th)

        # 1. TF (odom -> base_link) の配信
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        # 2. /odom トピックの配信
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # 位置 (Pose)
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # 速度 (Twist)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        self.odom_pub.publish(odom)

    def listener_callback(self, msg):
        # cmd_velの処理は変更なし
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        speed_a = vx - vy - (self.geo_factor * wz)
        speed_b = vx + vy + (self.geo_factor * wz)
        speed_c = vx + vy - (self.geo_factor * wz)
        speed_d = vx - vy + (self.geo_factor * wz)

        scale = self.pwm_limit
        m1 = speed_a * scale
        m2 = speed_b * scale
        m3 = speed_c * scale
        m4 = speed_d * scale

        self.stm.send_speeds(m1, m2, m3, m4)

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'read_thread'):
            self.read_thread.join()
        self.stm.send_speeds(0,0,0,0)
        self.stm.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    driver = MoebiusDriver()
    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        pass
    finally:
        driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
