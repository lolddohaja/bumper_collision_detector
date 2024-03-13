# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class BumperMonitorNode(Node):
    def __init__(self):
        super().__init__('bumper_monitor_node')
        self.publisher_ = self.create_publisher(String, 'collision', 10)
        self.ser = serial.Serial('/dev/ttyUSB0', 115200)
        self.timer = self.create_timer(0.1, self.check_collision)
        self.last_collision_level = 0  # 충돌 수준을 추적하기 위한 변수

    def check_collision(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').rstrip()
            self.get_logger().info(f'Received: {line}')
            try:
                value = int(line)
                if value > 0 and value != self.last_collision_level:
                    if value > 100:
                        self.report_collision(value)
                    if value > 150:
                        self.report_collision(value)
                    if value > 200:
                        self.report_collision(value)
                elif value < 0 and abs(value) > 100:  # 음수 값 처리
                    self.report_end_of_collision(abs(value))
            except ValueError:  # 숫자로 변환할 수 없는 값이 수신된 경우
                self.get_logger().error(f"Invalid data received: {line}")

    def report_collision(self, value):
        self.last_collision_level = value  # 충돌 수준 업데이트
        msg = String()
        msg.data = f"Collision detected with intensity: {value}"
        self.publisher_.publish(msg)

    def report_end_of_collision(self, value):
        self.last_collision_level = 0  # 충돌 수준 리셋
        msg = String()
        msg.data = f"Collision ended with negative intensity: {value}"
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BumperMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly')
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
