import rclpy
from rclpy.node import Node
import numpy as np

from time import sleep
from math import pi

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Bool

class TurnAroundNode(Node):
    def __init__(self):
        super().__init__('turn_around_node')

        self.active = False
        self.target_angle = 0
        
        self.create_timer(0.1, self.run_loop)
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.create_subscription(String, 'state', self.process_state, 10)

        self.completion_pub = self.create_publisher(Bool, 'turn_complete', 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def run_loop(self):

        if self.active:
            msg = Twist()

            print('test')

            # move back a little
            msg.linear.x = -0.1
            self.vel_pub.publish(msg)
            sleep(1)
            msg.linear.x = 0.0
            self.vel_pub.publish(msg)

            # turn to correct angle
            self.find_target_angle()
            turn_time = abs(self.target_angle / 180) * 10
            msg.angular.z = pi/10 if self.target_angle > 0 else -pi/10
            self.vel_pub.publish(msg)
            sleep(turn_time)
            msg.angular.z = 0.0
            self.vel_pub.publish(msg)

            # move set distance
            msg.linear.x = 0.1
            self.vel_pub.publish(msg)
            sleep(10)
            msg.linear.x = 0.0
            self.vel_pub.publish(msg)

            comp = Bool()
            comp.data = True

            self.completion_pub.publish(comp)
    
    def find_target_angle(self):

        max_sum = 0
        self.target_angle = 0

        for i in range(len(self.distances)-44):
            grouping = self.distances[i:i+45]
            max_sum = max(sum(grouping), max_sum)
            self.target_angle = self.angles[i+22] if sum(grouping) > max_sum else self.target_angle

        self.target_angle = self.target_angle if self.target_angle <= 180 else (self.target_angle-360)

    def process_scan(self, data):
        
        self.distances = np.array(data.ranges)
        self.angles = np.array(range(361))

        focus_area = np.where((self.distances > 0.1))
        self.distances = self.distances[focus_area]
        self.angles = self.angles[focus_area]

        if min(self.distances) > 1:
            comp = Bool()
            comp.data = True
            self.completion_pub.publish(comp)

    def process_state(self, msg: String):
        if msg.data == 'Spiral':
            self.active = True
        elif msg.data != None and len(msg.data) > 2:
            self.active = False
            comp = Bool()
            comp.data = False
            self.completion_pub.publish(comp)

def main(args=None):
    rclpy.init(args=args)
    node = TurnAroundNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()