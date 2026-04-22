### 上位机ros2代码
```
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading
import time

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')

        self.PORT = '/dev/ttyUSB0'
        self.BAUDRATE = 115200

        self.MOTOR_MICROSTEPS = 3200
        self.GEAR_RATIO = 15
        self.PULSES_PER_REV = self.MOTOR_MICROSTEPS * self.GEAR_RATIO

        self.SPROCKET_TEETH = 40
        self.LINKS_TO_MOVE = 30

        self.TARGET_PULSES = int((self.LINKS_TO_MOVE / self.SPROCKET_TEETH) * self.PULSES_PER_REV)

        self.TOTAL_TRAYS = 5
        self.current_tray_index = 0

        self.serial_lock = threading.Lock()

        self.get_logger().info(f"尝试连接串口 {self.PORT} ...")
        try:
            self.ser = serial.Serial(
                self.PORT,
                self.BAUDRATE,
                timeout=0.1,
                rtscts=False,
                dsrdtr=False
            )
            self.get_logger().info("串口连接成功")
            self.get_logger().info(f"齿数={self.SPROCKET_TEETH}, 减速比=1:{self.GEAR_RATIO}")
            self.get_logger().info(f"单次步进脉冲: {self.TARGET_PULSES}")
        except Exception as e:
            self.get_logger().error(f"串口打开失败: {e}")
            exit()

        self.sub_cmd = self.create_subscription(String, '/motor_cmd', self.cmd_callback, 10)
        self.pub_status = self.create_publisher(String, '/motor_status', 10)
        self.heartbeat_timer = self.create_timer(1.0, self.heartbeat_callback)

        self.receive_thread = threading.Thread(target=self.receive_task, daemon=True)
        self.receive_thread.start()

    def send_command(self, cmd, arr_val):
        arr_h = (int(arr_val) >> 8) & 0xFF
        arr_l = int(arr_val) & 0xFF
        checksum = (cmd + arr_h + arr_l) & 0xFF
        packet = [0xAA, 0x55, cmd, arr_h, arr_l, checksum, 0xFF]

        try:
            with self.serial_lock:
                self.ser.write(bytes(packet))
                self.ser.flush()
        except Exception as e:
            self.get_logger().error(f'串口发送异常: {e}')

    def cmd_callback(self, msg):
        user_input = msg.data.strip().upper()

        if 'N' in user_input:
            self.send_command(0x05, self.TARGET_PULSES)
            self.get_logger().info(f"执行换位: {self.current_tray_index+1}/{self.TOTAL_TRAYS} 脉冲: {self.TARGET_PULSES}")
            self.current_tray_index = (self.current_tray_index + 1) % self.TOTAL_TRAYS

        elif 'S' in user_input:
            self.send_command(0x03, 0)
            self.get_logger().warn("紧急刹车")

    def heartbeat_callback(self):
        self.send_command(0x00, 0)

    def receive_task(self):
        buffer = bytearray()
        while rclpy.ok():
            try:
                if self.ser.in_waiting > 0:
                    buffer.extend(self.ser.read(self.ser.in_waiting))
                    while len(buffer) >= 8:
                        if buffer[0] == 0xAA and buffer[1] == 0x55 and buffer[2] == 0x81:
                            calc_sum = (buffer[2] + buffer[3] + buffer[4] + buffer[5]) & 0xFF
                            if calc_sum == buffer[6] and buffer[7] == 0xFF:
                                motor_state = buffer[3]
                                current_speed = (buffer[4] << 8) | buffer[5]

                                state_str = "停机待命"
                                if motor_state == 1:
                                    state_str = "正在加速"
                                elif motor_state == 2:
                                    state_str = "匀速巡航"
                                elif motor_state == 3:
                                    state_str = "正在减速"
                                elif motor_state == 4:
                                    state_str = "精准定位中"

                                status_msg = String()
                                status_msg.data = f"状态: {state_str} | 数值: {current_speed:04d}"
                                self.pub_status.publish(status_msg)

                                buffer = buffer[8:]
                            else:
                                buffer.pop(0)
                        else:
                            buffer.pop(0)
            except Exception:
                pass
            time.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('程序中断')
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

指令：

ros2 topic pub --once /motor_cmd std_msgs/msg/String "{data: N}"


ros2 topic pub --once /motor_cmd std_msgs/msg/String "{data: S}"

ros2 topic echo /motor_status