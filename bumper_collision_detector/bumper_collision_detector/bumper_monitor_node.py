# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class BumperMonitorNode(Node):
    def __init__(self):
        super().__init__('bumper_monitor_node')

        self.collision_threshold = self.declare_parameter('collision_value', 100).get_parameter_value().integer_value
        self.end_collision_threshold = self.declare_parameter('end_collision_value', -50).get_parameter_value().integer_value

        self.publisher_ = self.create_publisher(String, 'collision', 10)
        self.ser = serial.Serial('/dev/ttyUSB0', 115200)
        self.timer = self.create_timer(0.1, self.check_collision)
        self.last_collision_level = 0  # 충돌 수준을 추적하기 위한 변수

    def check_collision(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').rstrip()
            # self.get_logger().info(f'Received: {line}')
            try:
                value = int(line)
                if value > self.collision_threshold and value != self.last_collision_level:
                    self.report_collision(value)
                elif value < self.end_collision_threshold:  
                    self.report_end_of_collision(value)  # 음수 값 그대로 전달
            except ValueError:  # 숫자로 변환할 수 없는 값이 수신된 경우
                self.get_logger().error(f"Invalid data received: {line}")

    def report_collision(self, value):
        # 충돌 발생 시
        msg = String()
        msg.data = f"is_collision:True,intensity:{value}"
        self.publisher_.publish(msg)
        self.last_collision_level = value  # 충돌 수준 업데이트

    def report_end_of_collision(self, value):
        # 충돌 종료 시
        msg = String()
        msg.data = f"is_collision:False,intensity:{value}"
        self.publisher_.publish(msg)
        self.last_collision_level = 0  # 충돌 수준 리셋

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
