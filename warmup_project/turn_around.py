import rclpy
from rclpy.node import Node
import numpy as np
from time import time

from time import sleep
from math import inf, pi

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Bool


class TurnAroundNode(Node):
    """
    Node for executing a turn-around maneuver using laser scan data.
    Subscribes to 'scan' and 'state' topics, publishes velocity and completion status.
    This is a CoPilot-generated docstring (that we checked for accuracy).
    """

    def __init__(self):
        """Initializes the node"""
        super().__init__('turn_around_node')

        self.active = False
        self.target_angle = 0

        # time turn_around starts (for timing movements without sleep)
        self.start_time = 0.0

        self.create_timer(0.1, self.run_loop)
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.create_subscription(String, 'state', self.process_state, 10)

        self.completion_pub = self.create_publisher(Bool, 'turn_complete', 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def run_loop(self):
        """
        Main loop for turn-around behavior. Executes a sequence of movements: backup, 
        turn, move forward, and signals completion.
        This is a CoPilot-generated docstring (that we checked for accuracy).
        """

        if self.active:
            msg = Twist()

            turn_time = abs(self.target_angle / 180) * 10

            # move back a little
            if abs(self.start_time - time()) < 2:
                msg.linear.x = -0.15
                msg.angular.z = 0.0
                self.vel_pub.publish(msg)
                self.find_target_angle()

            # turn to correct angle
            elif abs(self.start_time - time()) < 2+turn_time:
                msg.linear.x = 0.0
                if self.target_angle > 0:
                    msg.angular.z = pi/10
                else:
                    msg.angular.z = -pi/10

                print(f"turn time: {turn_time}")
                print(f"target angle: {self.target_angle}")
                print(f"angular velocity: {msg.angular.z}")

                self.vel_pub.publish(msg)

            # move set distance
            elif abs(self.start_time - time()) < 2+turn_time+10:
                msg.linear.x = 0.15
                msg.angular.z = 0.0
                self.vel_pub.publish(msg)

            else:
                comp = Bool()
                comp.data = True
                self.completion_pub.publish(comp)

    def find_target_angle(self):
        """
        Finds the angle corresponding to the closest 45 degree cone of points in the scan data.
        Updates self.target_angle.
        This is a CoPilot-generated docstring (that we checked for accuracy).
        """

        max_sum = 0
        self.target_angle = 0

        for i in range(len(self.distances)-44):
            grouping = self.distances[i:i+45]
            self.target_angle = self.angles[i+22] if sum(
                grouping) > max_sum else self.target_angle
            max_sum = max(sum(grouping), max_sum)

        self.target_angle = self.target_angle if self.target_angle <= 180 else (
            self.target_angle-360)

    def process_scan(self, data):
        """
        Callback for LaserScan messages. Updates distances and angles arrays, 
        replaces inf values, and focuses on valid scan data.
        This is a CoPilot-generated docstring (that we checked for accuracy).

        Args:
            data (LaserScan): 360 degree distance scans
        """

        self.distances = np.array(data.ranges)
        self.angles = np.array(range(361))

        # no max since we're trying to find the furthest open area
        focus_area = np.where((self.distances > 0.1))
        self.distances = self.distances[focus_area]
        self.angles = self.angles[focus_area]

        # replace inf values with 100 for neeto sim
        for i in range(0, len(self.distances)):
            if self.distances[i] == inf:
                self.distances[i] = 100

    def process_state(self, msg: String):
        """
        Callback for state messages. Activates or deactivates turn-around behavior based on state.
        Publishes completion status.
        This is a CoPilot-generated docstring (that we checked for accuracy).

        Args:
            data (LaserScan): 360 degree distance scans
        """
        if msg.data == 'Turn Around':
            self.active = True
            self.start_time = time()
            comp = Bool()
            comp.data = False
            self.completion_pub.publish(comp)
        elif msg.data != None and len(msg.data) > 2:
            self.active = False
            comp = Bool()
            comp.data = False
            self.completion_pub.publish(comp)


def main(args=None):
    """
    Initializes TurnAroundNode
    This is a CoPilot-generated docstring (that we checked for accuracy).
    """
    rclpy.init(args=args)
    node = TurnAroundNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
