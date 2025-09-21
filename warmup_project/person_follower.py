import rclpy
from rclpy.node import Node
import numpy as np

from time import sleep
from math import pi

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Bool

class PersonFollowerNode(Node):
    def __init__(self):
        super().__init__('person_follower_node')

        self.active = False
        self.target_angle = 0
        self.degs_to_rads = np.pi / 180
        self.distances = np.array(range(361))
        self.angles = np.array(range(361))

        self.prev_ta = 10000
        
        self.create_timer(0.1, self.run_loop)
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.create_subscription(String, 'state', self.process_state, 10)

        self.completion_pub = self.create_publisher(Bool, 'full_empty', 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def run_loop(self):
        if len(self.angles) > 0 and self.active:

            self.find_target_angle()

            # --- make Twist message ---
            msg = Twist()

            print(self.target_angle)

            if self.target_angle != self.prev_ta:
                if (self.target_angle < 45 and self.target_angle > -45):
                    msg.linear.x = 0.05
                    msg.angular.z = 0.2 * (self.target_angle / 45)
                    self.vel_pub.publish(msg)
                else:
                    msg.linear.x = 0.0
                    turn_time = abs(self.target_angle / 180) * 10
                    msg.angular.z = pi/10 if self.target_angle > 0 else -pi/10
                    self.vel_pub.publish(msg)
                    sleep(turn_time)
                    msg.angular.z = 0.0
                    self.vel_pub.publish(msg)
            
            self.prev_ta = self.target_angle

        else:
            # stop if not active or no target
            stop_msg = Twist()
            self.vel_pub.publish(stop_msg)

    def find_target_angle(self):

        min_sum = 1000000000.0
        self.target_angle = 0

        for i in range(len(self.distances)-44):
            grouping = self.distances[i:i+45]
            self.target_angle = self.angles[i+22] if sum(grouping) < min_sum else self.target_angle
            min_sum = min(sum(grouping), min_sum)

        self.target_angle = self.target_angle if self.target_angle <= 180 else (self.target_angle-360)

    def process_scan(self, data):
        #gets locations of objects

        self.distances = np.array(data.ranges)
        self.angles = np.array(range(361))

        focus_area = np.where((self.distances > 0.1) & (self.distances < 2))
        self.distances = self.distances[focus_area]
        self.angles = self.angles[focus_area]

    def process_state(self, msg: String):
        if msg.data == 'Person Follower':
            self.active = True
        else:
            self.active = False


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()