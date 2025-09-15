import rclpy
from rclpy.node import Node
import numpy as np

from math import pi

from neato2_interfaces.msg import Bump
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class circle_node(Node):
    """This has our neato go in a circle"""
    def __init__(self):
        """Initializes the circle_node"""
        
        super().__init__('circle_node')

        timer_period = 0.1

        self.speed = 0.1
        self.circle_time = rclpy.time.Duration(seconds=40)

        self.start_time = None

        self.timer = self.create_timer(timer_period, self.run_loop)
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

    def run_loop(self):
        """draws a circle"""
        vel = Twist()

        if self.start_time is None:
            self.start_time = self.get_clock().now()
        
        current_time = self.get_clock().now()
        time_since_start = current_time - self.start_time
        if (time_since_start < self.circle_time):
            vel.linear.x = self.speed
            vel.angular.z = pi / 20
        else:
            vel.linear.x = 0.0
            vel.angular.z = 0.0

        self.vel_pub.publish(vel)

def main(args=None):
    """Initialize node. Run node. clean up after termination"""
    rclpy.init(args=args) # Initializing communicating with ROS
    node = circle_node() # Make node
    rclpy.spin(node) # runs node until interruption
    rclpy.shutdown()


if __name__ == '__main__':
    main()