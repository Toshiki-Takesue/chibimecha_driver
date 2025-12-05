import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

# --- STM32通信クラス ---
class STM32Interface:
    def __init__(self, port, baud=115200):
        self.port = port
        self.baud = baud
        self.ser = None
        # 接続はあとで行うのでここでは呼ばない（ロガー設定後に行うため）

    def connect(self):
        try:
            # ポートを開く
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            
            # --- リセット解除 ---
            self.ser.rts = False
            self.ser.dtr = False
            time.sleep(0.1)
            self.ser.dtr = True  # DTR ON
            time.sleep(0.1)
            self.ser.dtr = False # DTR OFF
            
            # 起動待ち
            time.sleep(2.0)
            self.ser.reset_input_buffer()
            return True
            
        except Exception as e:
            return False, e

    def send_speeds(self, m1, m2, m3, m4):
        if self.ser and self.ser.is_open:
            # コマンド送信
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
        
        # 1. パラメータの宣言（デフォルト値を設定）
        self.declare_parameter('pwm_limit', 150)
        self.declare_parameter('port', '/dev/ttyUSB0')  # ★ここを追加

        # 2. パラメータの取得
        self.pwm_limit = self.get_parameter('pwm_limit').value
        port_name = self.get_parameter('port').value    # ★設定されたポート名を取得

        # 3. STM32との接続
        self.get_logger().info(f"Connecting to {port_name} ...")
        self.stm = STM32Interface(port_name, 115200)
        
        # 接続試行
        success = self.stm.connect()
        if success is True:
            self.get_logger().info("Connected to STM32 successfully!")
        else:
            # 失敗した場合はエラーログを出す(successにエラー内容が入っている)
            self.get_logger().error(f"Connection Failed: {success[1]}")

        # 速度指令 (/cmd_vel) を購読
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        
        self.get_logger().info(f"Moebius Driver Started (PWM Limit: {self.pwm_limit})")

    def listener_callback(self, msg):
        # 1. ROSからの指令
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        # 2. メカナムホイール計算
        speed_a = vx - vy - wz
        speed_b = vx + vy + wz
        speed_c = vx + vy - wz
        speed_d = vx - vy + wz

        # 3. PWM変換
        scale = self.pwm_limit
        m1 = speed_a * scale
        m2 = speed_b * scale
        m3 = speed_c * scale
        m4 = speed_d * scale

        # 4. 送信
        self.stm.send_speeds(m1, m2, m3, m4)

    def destroy_node(self):
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