#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import threading

class TrajectoryPlotter(Node):
    """
    Скрипт-анализатор для построения графиков движения на основе одометрии (Подраздел 3.2).
    Подписывается на топик /diff_drive_controller/odom и сохраняет траекторию (X, Y).
    """
    def __init__(self):
        super().__init__('trajectory_plotter')
        self.subscription = self.create_subscription(
            Odometry,
            '/diff_drive_controller/odom',
            self.odom_callback,
            10)
        self.x_data = []
        self.y_data = []
        self.lock = threading.Lock()
        self.get_logger().info("Trajectory Plotter Started. Collecting data...")

    def odom_callback(self, msg):
        with self.lock:
            self.x_data.append(msg.pose.pose.position.x)
            self.y_data.append(msg.pose.pose.position.y)

def plot_data(node):
    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 6))
    line, = ax.plot([], [], 'b-', label='Simulated Odometry (with slip)')
    ax.set_xlabel('X [m]', fontsize=12)
    ax.set_ylabel('Y [m]', fontsize=12)
    ax.set_title('Robot Trajectory Comparison', fontsize=14)
    ax.grid(True)
    ax.legend(loc='best')
    
    # Enable tight layout for proper saving
    fig.tight_layout()

    # Pre-set limits (can be dynamic, but this is simpler for typical scenarios)
    ax.set_xlim(-5, 5)
    ax.set_ylim(-3, 7)

    try:
        while rclpy.ok():
            with node.lock:
                if node.x_data:
                    line.set_xdata(node.x_data)
                    line.set_ydata(node.y_data)
                    # Dynamically adjust limits if needed
                    ax.relim()
                    ax.autoscale_view()
            fig.canvas.draw()
            fig.canvas.flush_events()
            plt.pause(0.1)
    except Exception as e:
        node.get_logger().error(f"Plotting error: {e}")
    finally:
        # Save figure upon exit
        plt.ioff()
        if node.x_data:
             filename = '/home/dev/ws/src/sim2d_hardware_interface/scripts/trajectory_result.png'
             plt.savefig(filename, dpi=300)
             print(f"\n[plot_trajectory] Final plot saved to {filename}")
        plt.close(fig)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlotter()
    
    # Run ros spin in a separate thread so pyplot can run in the main thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()
    
    try:
        plot_data(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()
