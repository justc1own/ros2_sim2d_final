#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class TeleopScenario(Node):
    """
    Автоматизированный сценарий управления для валидации кинематических моделей (Подраздел 3.1, 3.2).
    Публикует последовательность Twist сообщений для генерации заданных траекторий.
    """
    def __init__(self):
        super().__init__('teleop_scenario')
        self.publisher_ = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.start_time_ = time.time()
        
        # Scenario states: 0: Circle, 1: Figure-8, 2: Stop
        self.state_ = 0
        self.get_logger().info("Teleop Scenario Node Started. Executing Circle -> Figure 8.")

    def timer_callback(self):
        msg = Twist()
        current_time = time.time() - self.start_time_

        if self.state_ == 0:
            # Circle (radius ~ 2m): V = 1.0 m/s, W = 0.5 rad/s -> R = V/W = 2m
            msg.linear.x = 1.0
            msg.angular.z = 0.5
            if current_time > 15.0: # ~ 1.2 revolutions
                self.state_ = 1
                self.start_time_ = time.time()
                self.get_logger().info("Switching to Figure-8 Scenario")
                
        elif self.state_ == 1:
            # Figure-8 (approximate using sine functions)
            msg.linear.x = 1.0
            msg.angular.z = 1.0 * math.sin(current_time * 0.5)
            if current_time > 30.0:
                self.state_ = 2
                self.get_logger().info("Scenario Completed. Stopping.")
                
        elif self.state_ == 2:
            # Stop
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopScenario()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
