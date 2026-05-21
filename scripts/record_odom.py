#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import time
import os

class OdomRecorder(Node):
    """
    Скрипт для записи одометрии в CSV файл для последующего анализа в Jupyter.
    """
    def __init__(self):
        super().__init__('odom_recorder')
        self.subscription = self.create_subscription(
            Odometry,
            '/diff_drive_controller/odom',
            self.odom_callback,
            10)
        
        # Динамический путь относительно директории самого скрипта:
        self.csv_path = os.path.join(os.path.dirname(__file__), 'trajectory_data.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'x', 'y'])
        self.start_time = time.time()
        
        self.get_logger().info(f"Started recording odometry to {self.csv_path}")

    def odom_callback(self, msg):
        t = time.time() - self.start_time
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.csv_writer.writerow([t, x, y])
        
        # ПРИНУДИТЕЛЬНЫЙ СБРОС БУФЕРА НА ЖЕСТКИЙ ДИСК
        self.csv_file.flush()
        # os.fsync(self.csv_file.fileno()) # Опционально: можно добавить для 100% гарантии на уровне ОС

    def destroy_node(self):
        self.csv_file.close()
        self.get_logger().info("Recording saved and closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OdomRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt caught, trying to shutdown gracefully...")
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()