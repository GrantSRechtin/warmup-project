import rclpy
from rclpy.node import Node
import numpy as np

from time import time
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

        self.num = 0
        self.prev_num = 0

        self.start_time = 0.0
        self.turn_time = 0.0

        self.big_turn = False
        
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

            if self.num != self.prev_num:
                if self.big_turn and abs(time() - self.start_time) < self.turn_time:
                    self.big_turn = False
                elif not self.big_turn:
                    if (self.target_angle < 45 and self.target_angle > -45):
                        msg.linear.x = 0.1
                        msg.angular.z = 0.2 * (self.target_angle / 45)
                        self.vel_pub.publish(msg)
                    else:

                        self.start_time = time()

                        self.big_turn = True
                        msg.linear.x = 0.0
                        self.turn_time = abs(self.target_angle / 180) * 10
                        if self.target_angle > 0:
                            msg.angular.z = pi/10
                        else:
                            msg.angular.z = -pi/10
                        
                        self.vel_pub.publish(msg)
            
            self.prev_num = self.num

    def find_target_angle(self):

        min_sum = 1000000000.0
        self.target_angle = 0

        for i in range(len(self.distances)-15):
            grouping = self.distances[i:i+15]
            self.target_angle = self.angles[i+8] if sum(grouping) < min_sum else self.target_angle
            min_sum = min(sum(grouping), min_sum)

        self.target_angle = self.target_angle if self.target_angle <= 180 else (self.target_angle-360)

    def process_scan(self, data):
        #gets locations of objects
        self.num += 1

        self.distances = np.array(data.ranges)
        self.angles = np.array(range(361))

        focus_area = np.where((self.distances > 0.1) & (self.distances < 1))
        self.distances = self.distances[focus_area]
        self.angles = self.angles[focus_area]

        self.check_full_empty()

    def process_state(self, msg: String):
        if msg.data == 'Person Following':
            self.active = True
        elif msg.data != None and len(msg.data) > 2:
            self.active = False

    def check_full_empty(self):
        if len(self.distances) > 1 and min(self.distances) < 0.5:
            fe = Bool()
            fe.data = True
            self.completion_pub.publish(fe)
        else:
            fe = Bool()
            fe.data = False
            self.completion_pub.publish(fe)


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()