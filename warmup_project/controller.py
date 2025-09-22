import rclpy
from rclpy.node import Node
import numpy as np

from math import pi
from time import sleep
from time import time

from neato2_interfaces.msg import Bump
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_msgs.msg import Bool


class ControllerNode(Node):
    """
    ROS2 Node that manages robot behavior states (Spiral, Person Following, Turn Around)
    based on sensor inputs (LaserScan, Bump, Full/Empty, Turn Complete).
    Publishes state changes and velocity commands to control robot movement.
    This is a CoPilot-generated docstring (that we checked for accuracy).
    """

    def __init__(self):
        """
        Initialize ControllerNode, set up publishers, subscriptions, timers, and initial state.
        This is a CoPilot-generated docstring (that we checked for accuracy).
        """
        super().__init__('controller_node')

        self.distances = np.array(range(361))
        self.angles = np.array(range(361))

        self.bump_cooldown_start = 0

        self.bumped = False
        self.full_empty = False
        self.turn_complete = False

        self.state = 'Spiral'
        # for detecting followable obj
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.create_subscription(Bump, 'bump', self.process_bump, 10)
        self.create_subscription(
            Bool, 'full_empty', self.process_full_empty, 10)
        self.create_subscription(
            Bool, 'turn_complete', self.process_turn_complete, 10)
        self.state_publisher = self.create_publisher(String, 'state', 10)
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.init_msg = String()
        self.init_msg.data = self.state

        self.create_timer(0.1, self.run_loop)
        self.state_publisher.publish(self.init_msg)

    def run_loop(self):
        """
        Main control loop. Evaluates sensor states and transitions robot behavior state.
        Publishes state changes and stops robot when transitioning.
        This is a CoPilot-generated docstring (that we checked for accuracy).
        """
        msg = String()

        print(self.state)

        # State transition logic based on sensor inputs
        if self.state == 'Person Following' and self.bumped and abs(time() - self.bump_cooldown_start) > 2:
            # If PF but hit something, turn around.
            msg.data = 'Turn Around'
            self.state = msg.data
            self.state_publisher.publish(msg)
            self.bumped = False
            t = Twist()
            t.angular.z = 0.0
            t.linear.x = 0.0
            self.vel_publisher.publish(t)
            self.bump_cooldown_start = time()
            self.turn_complete = False
        elif self.state == 'Person Following' and self.full_empty and not self.check_follow_people():
            # If PF but no person to follow and theres a bunch of objects, turn around.
            msg.data = 'Turn Around'
            self.state = msg.data
            self.state_publisher.publish(msg)
            t = Twist()
            t.angular.z = 0.0
            t.linear.x = 0.0
            self.vel_publisher.publish(t)
        elif self.state == 'Person Following' and not self.full_empty and not self.check_follow_people():
            # If PF but no person to follow and theres nothing around, start spiraling.
            msg.data = 'Spiral'
            self.state = msg.data
            self.state_publisher.publish(msg)
            t = Twist()
            t.angular.z = 0.0
            t.linear.x = 0.0
            self.vel_publisher.publish(t)
        elif self.state == 'Turn Around' and self.bumped and abs(time() - self.bump_cooldown_start) > 2:
            # If TA but hit something, try again.
            msg.data = 'Turn Around'
            self.state = msg.data
            self.state_publisher.publish(msg)
            self.bumped = False
            t = Twist()
            t.angular.z = 0.0
            t.linear.x = 0.0
            self.vel_publisher.publish(t)
            self.bump_cooldown_start = time()
        elif self.state == 'Turn Around' and self.turn_complete and not self.check_follow_people():
            # If TA and done turning but theres no one to follow, spiral
            msg.data = 'Spiral'
            self.state = msg.data
            self.state_publisher.publish(msg)
            t = Twist()
            t.angular.z = 0.0
            t.linear.x = 0.0
            self.vel_publisher.publish(t)
        elif self.state == 'Turn Around' and self.turn_complete and self.check_follow_people():
            # If TA and done turning and there's someone to follow, follow
            msg.data = 'Person Following'
            self.state = msg.data
            self.state_publisher.publish(msg)
            t = Twist()
            t.angular.z = 0.0
            t.linear.x = 0.0
            self.vel_publisher.publish(t)
        elif self.state == 'Spiral' and self.check_follow_people():
            # If spiraling but found someone to follow, follow
            msg.data = 'Person Following'
            self.state = msg.data
            self.state_publisher.publish(msg)
            t = Twist()
            t.angular.z = 0.0
            t.linear.x = 0.0
            self.vel_publisher.publish(t)
        elif self.state == 'Spiral' and self.bumped and abs(time() - self.bump_cooldown_start) > 2:
            # If spiraling but hit something, turn around
            msg.data = 'Turn Around'
            self.state = msg.data
            self.state_publisher.publish(msg)
            self.bumped = False
            t = Twist()
            t.angular.z = 0.0
            t.linear.x = 0.0
            self.vel_publisher.publish(t)
            self.bump_cooldown_start = time()

    def process_scan(self, data):
        """
        Callback for LaserScan topic. Updates distances and angles arrays with detected objects.
        Filters objects within 0.1m to 1m range.
        This is a CoPilot-generated docstring (that we checked for accuracy).

        Args:
            data (LaserScan): Incoming laser scan data.
        """

        self.distances = np.array(data.ranges)
        self.angles = np.array(range(361))

        focus_area = np.where((self.distances > 0.1) & (self.distances < 1))
        self.distances = self.distances[focus_area]
        self.angles = self.angles[focus_area]

    def check_follow_people(self):
        """
        Callback for LaserScan topic. Updates distances and angles arrays with detected objects.
        Filters objects within 0.1m to 1m range.
        This is a CoPilot-generated docstring (that we checked for accuracy).

        Args:
            data (LaserScan): Incoming laser scan data.
        """

        ang = self.angles
        dist = self.distances

        if len(dist) > 0 and len(dist) < 361:

            closest_d = sum(dist[np.where(dist == np.min(dist))]) / \
                len(dist[np.where(dist == np.min(dist))])
            # closest_a = ang[np.where(dist == np.min(dist))]
            dist_range = np.where((dist < closest_d+0.1))

            print(len(ang[dist_range]))

            not_front_range = np.where((ang > 45) & (ang < 315))
            not_front = dist[not_front_range]

            if (abs(np.max(ang[dist_range]) - np.min(ang[dist_range])) <= 45) or ((np.max(ang[dist_range]) >= 315) & (np.min(ang[dist_range]) <= 45) & len(not_front) == 0):
                return True
            else:
                return False
        else:
            return False

    def process_bump(self, msg):
        """
    Callback for bump sensor input. Sets bumped state if any bump sensor is triggered.
    This is a CoPilot-generated docstring (that we checked for accuracy).

    Args:
        msg (Bump): Bump message from subscriber.
    """

        if (msg.left_front == 1 or
                msg.right_front == 1 or
                msg.left_side == 1 or
                msg.right_side == 1):
            self.bumped = True
        else:
            self.bumped = False

    def process_full_empty(self, msg):
        """
        Callback for full_empty topic. Updates full_empty state.
        This is a CoPilot-generated docstring (that we checked for accuracy).

        Args:
            msg (Bool): Full/empty message.
        """
        self.full_empty = msg.data

    def process_turn_complete(self, msg):
        """
        Callback for turn_complete topic. Updates turn_complete state.
        This is a CoPilot-generated docstring (that we checked for accuracy).

        Args:
            msg (Bool): Turn complete message.
        """
        self.turn_complete = msg.data


def main(args=None):
    """
    Entry point for the controller node.
    Initializes ROS, creates the node, and spins until interrupted.
    This is a CoPilot-generated docstring (that we checked for accuracy).
    """
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
