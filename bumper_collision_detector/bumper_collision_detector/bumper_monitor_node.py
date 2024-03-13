# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class BumperMonitorNode(Node):
    def __init__(self):
        super().__init__('bumper_monitor_node')
        # 파라미터로부터 임계값을 가져옵니다.
        self.collision_threshold = self.declare_parameter('collision_value', 100).get_parameter_value().integer_value
        self.end_collision_threshold = self.declare_parameter('end_collision_value', -80).get_parameter_value().integer_value
        self.end_positive_count = self.declare_parameter('end_positive_count', 10).get_parameter_value().integer_value

        self.publisher_ = self.create_publisher(String, 'collision', 10)
        self.ser = serial.Serial('/dev/ttyUSB0', 115200)
        self.timer = self.create_timer(0.1, self.check_collision)
        self.last_value = 0  # 마지막 센서 값
        self.is_in_collision = False  # 현재 충돌 상태
        self.reached_below_threshold = False

    def check_collision(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').rstrip()
            try:
                current_value = int(line)

                # 충돌이 시작될 때만 메시지를 발행
                if current_value > self.collision_threshold and not self.is_in_collision:
                    self.report_collision(current_value)
                    self.is_in_collision = True  # 충돌 상태로 전환
                    self.reached_below_threshold = False  # -80 이하로 내려간 적이 있는지 플래그 초기화
                    self.positive_count = 0  # 양수 값 카운트 초기화

                # 센서 값이 self.end_collision_threshold 이하로 떨어진 경우 플래그 설정
                if current_value < self.end_collision_threshold:
                    self.reached_below_threshold = True

                # 센서 값이 양수이고, -80 이하로 내려간 적이 있으면 카운트 증가
                if current_value >= 0 and self.reached_below_threshold:
                    self.positive_count += 1
                else:
                    self.positive_count = 0  # 연속 조건이 깨지면 카운트 초기화

                # 양수 값이 10번 연속되고, 충돌 상태인 경우 충돌이 끝났다고 판단
                if self.positive_count >= 10 and self.is_in_collision:
                    self.report_end_of_collision(current_value)
                    self.is_in_collision = False  # 충돌 상태 해제
                    self.reached_below_threshold = False  # 플래그 초기화
                    self.positive_count = 0  # 양수 값 카운트 초기화

                self.last_value = current_value  # 마지막 값 업데이트

                # self.get_logger().info(f"Current value: {current_value}")

            except ValueError:
                self.get_logger().error(f"Invalid data received: {line}")

    def report_collision(self, value):
        msg = String()
        msg.data = f"is_collision:True,intensity:{value}"
        self.publisher_.publish(msg)

    def report_end_of_collision(self, value):
        msg = String()
        msg.data = f"is_collision:False,intensity:{value}"
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
