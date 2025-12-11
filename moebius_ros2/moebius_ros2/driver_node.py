import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import serial
import time
import threading

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

    # 受信用のメソッドを追加（ブロッキングしないように）
    def read_line(self):
        if self.ser and self.ser.is_open and self.ser.in_waiting > 0:
            try:
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

        self.pwm_limit = self.get_parameter('pwm_limit').value
        port_name = self.get_parameter('port').value
        baud_rate = self.get_parameter('baud').value

        self.declare_parameter('tread', 0.255)
        self.declare_parameter('wheelbase', 0.230)
        tread = self.get_parameter('tread').value
        wheelbase = self.get_parameter('wheelbase').value
        self.geo_factor = (tread + wheelbase) / 2.0

        # パブリッシャ (IMU用)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)

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

        # ★受信スレッド開始 (IMUデータを受け取るため)
        self.running = True
        self.read_thread = threading.Thread(target=self.read_loop)
        self.read_thread.start()
        
        self.get_logger().info(f"Moebius Driver Started (Motor + IMU).")

    # 別スレッドで常にシリアルを監視する
    def read_loop(self):
        while self.running and rclpy.ok():
            line = self.stm.read_line()
            if line:
                # "Alive! IMU: AcX=..." というデータを探す
                if "IMU" in line:
                    self.parse_imu(line)
            time.sleep(0.005)

    # 文字列を解析してROSメッセージにする
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
                
                # 生データを物理量に変換 (例: MPU6050の感度設定による)
                imu_msg.angular_velocity.z = data['GyZ'] / 131.0 
                if 'AcX' in data: imu_msg.linear_acceleration.x = data['AcX'] / 16384.0 * 9.8
                if 'AcY' in data: imu_msg.linear_acceleration.y = data['AcY'] / 16384.0 * 9.8
                
                self.imu_pub.publish(imu_msg)
        except:
            pass

    def listener_callback(self, msg):
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