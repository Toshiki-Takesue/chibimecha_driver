import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
import time
import threading
import math

# --- STM32通信クラス ---
class STM32Interface:
    def __init__(self, port, baud=460800):
        self.port = port
        self.baud = baud
        self.ser = None

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            self.ser.reset_input_buffer()
            return True
        except Exception as e:
            return False, e

    def read_line(self):
        if self.ser and self.ser.is_open and self.ser.in_waiting > 0:
            try:
                # エラーが出ても無視してデコード
                return self.ser.readline().decode('utf-8', errors='ignore').strip()
            except:
                return None
        return None

    def send_speeds(self, m1, m2, m3, m4):
        if self.ser and self.ser.is_open:
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
        self.declare_parameter('tread', 0.255)
        self.declare_parameter('wheelbase', 0.230)

        self.pwm_limit = self.get_parameter('pwm_limit').value
        port_name = self.get_parameter('port').value
        baud_rate = self.get_parameter('baud').value
        tread = self.get_parameter('tread').value
        wheelbase = self.get_parameter('wheelbase').value
        
        # メカナムホイールの幾何学係数
        self.geo_factor = (tread + wheelbase) / 2.0

        # パブリッシャ
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10) # ★追加: Odom用
        self.tf_broadcaster = TransformBroadcaster(self)             # ★追加: TF用

        # ロボットの状態変数 (オドメトリ計算用)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        # 直近の指令速度（これでオドメトリを計算します）
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0
        
        self.last_time = self.get_clock().now()

        # STM32接続
        self.get_logger().info(f"Connecting to {port_name} at {baud_rate}...")
        self.stm = STM32Interface(port_name, baud_rate)
        
        success = self.stm.connect()
        if success is True:
            self.get_logger().info("Connected to STM32 successfully!")
        else:
            self.get_logger().error(f"Connection Failed: {success[1]}")

        # サブスクライバ (cmd_vel)
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.listener_callback, 10)

        # 受信スレッド開始 (IMUデータ用)
        self.running = True
        self.read_thread = threading.Thread(target=self.read_loop)
        self.read_thread.start()

        # ★ここが重要: オドメトリを定期的に計算して配信するタイマー (20Hz)
        self.create_timer(0.05, self.update_odom_from_cmd)
        
        self.get_logger().info(f"Moebius Driver Started (Mode: Open-Loop Odom).")

    # --- オドメトリ計算部分 (エンコーダなし版) ---
    def update_odom_from_cmd(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # 指令速度(cmd_vel)通りに動いたと仮定して計算
        # ロボット座標系での移動量
        delta_x = (self.cmd_vx * math.cos(self.th) - self.cmd_vy * math.sin(self.th)) * dt
        delta_y = (self.cmd_vx * math.sin(self.th) + self.cmd_vy * math.cos(self.th)) * dt
        delta_th = self.cmd_wz * dt

        # 座標更新
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # クォータニオン変換
        q = self.euler_to_quaternion(0, 0, self.th)

        # 1. TF配信 (odom -> base_footprint)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        # 2. Odomトピック配信
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        
        # 速度情報も指令値をそのまま入れる
        odom.twist.twist.linear.x = self.cmd_vx
        odom.twist.twist.linear.y = self.cmd_vy
        odom.twist.twist.angular.z = self.cmd_wz
        
        self.odom_pub.publish(odom)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    # --- 受信ループ (STM32からのIMUデータ受け取り) ---
    def read_loop(self):
        while self.running and rclpy.ok():
            line = self.stm.read_line()
            if line:
                # "Alive! IMU: AcX=..." を解析
                if "IMU" in line:
                    self.parse_imu(line)
            time.sleep(0.005)

    def parse_imu(self, line):
        try:
            parts = line.split('|')
            data = {}
            for p in parts:
                p = p.strip()
                if '=' in p:
                    key, val = p.split('=')
                    key = key.split(' ')[-1]
                    data[key] = float(val)

            if 'GyZ' in data:
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = "imu_link"
                
                # 生データを物理量に変換
                imu_msg.angular_velocity.z = (data['GyZ'] / 131.0) * (3.14159 / 180.0)
                
                if 'AcX' in data: imu_msg.linear_acceleration.x = (data['AcX'] / 16384.0) * 9.80665
                if 'AcY' in data: imu_msg.linear_acceleration.y = (data['AcY'] / 16384.0) * 9.80665
                
                self.imu_pub.publish(imu_msg)
        except:
            pass

    # --- コールバック (cmd_vel受信) ---
    def listener_callback(self, msg):
        # 1. 指令速度を保存 (オドメトリ計算用)
        self.cmd_vx = msg.linear.x
        self.cmd_vy = msg.linear.y
        self.cmd_wz = msg.angular.z

        # 2. メカナムホイールの各モーター速度を計算
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

        # 3. STM32へ送信
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
