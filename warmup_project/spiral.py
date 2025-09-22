import rclpy
from rclpy.node import Node
import numpy as np

from math import pi
from time import sleep

from geometry_msgs.msg import Twist
from std_msgs.msg import String

class SpiralNode(Node):
    """
    ROS2 Node that publishes velocity commands to move a robot in an expanding spiral pattern.
    Listens for state messages to activate or deactivate spiral movement.
    This is a CoPilot-generated docstring (that we checked for accuracy).
    """
    def __init__(self):
        """
        Initialize the SpiralNode, set up publishers, subscriptions, timers, and initial parameters.
        This is a CoPilot-generated docstring (that we checked for accuracy).
        """
        super().__init__('spiral_node')

        self.circle_time = rclpy.time.Duration(seconds=40)

        self.speed = 0.05
        self.angular_speed = 0.3

        self.start_time = None

        #Timer for motors
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)

        self.create_subscription(String, 'state', self.process_state, 10)

        self.active = False

        #Create the publisher for cmd_vel that tells the motors to move.
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

    def run_loop(self):
        """
        Timer callback that publishes velocity commands to move the robot in a spiral.
        Increases linear speed and decreases angular speed to expand the spiral.
        This is a CoPilot-generated docstring (that we checked for accuracy).
        """
        if self.active:
            """moves in an expanding spiral"""
            vel = Twist()

            self.speed *= 1.05
            vel.linear.x = self.speed

            self.angular_speed *= 0.95
            vel.angular.z = self.angular_speed

            self.vel_pub.publish(vel)
            print("vel published")
            sleep(5)

    def process_state(self, msg: String):
        """
        Callback for state topic. Activates spiral movement if state is 'Spiral'.
        Deactivates and resets parameters if other states.
        This is a CoPilot-generated docstring (that we checked for accuracy).
        
        Args:
            msg (String): Incoming state message.
        """
        if msg.data == 'Spiral':
            self.active = True
        elif msg.data != None and len(msg.data) > 2:
            self.active = False
            self.speed = 0.05
            self.angular_speed = 0.3

def main(args=None):
    """
    Entry point for the spiral node.
    Initializes ROS, creates the node, and runs until interrupted.
    This is a CoPilot-generated docstring (that we checked for accuracy).
    """
    rclpy.init(args=args) # Initializing communicating with ROS
    node = SpiralNode() # Make node
    rclpy.spin(node) # runs node until interruption
    rclpy.shutdown()


if __name__ == '__main__':
    main()
