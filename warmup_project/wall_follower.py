import rclpy
from rclpy.node import Node
import numpy as np

from math import pi

from neato2_interfaces.msg import Bump
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class wall_follower(Node):
    def __init__(self):
        super().__init__('wall_follower')

        self.front = 0.0

        self.lmean = 0.0
        self.lfmean = 0.0
        self.lbmean = 0.0

        self.rmean = 0.0
        self.rfmean = 0.0
        self.rbmean = 0.0

        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.create_timer(0.1, self.run_loop)
    
    def run_loop(self):
        msg = Twist()

        msg.linear.x = 0.1

        if self.lmean < self.rmean:

            if abs(self.lfmean-self.lbmean) < 0.01:
                msg.angular.z = 0.0
            else:
                msg.angular.z = (float) (self.lfmean - self.lbmean)

        elif self.rmean < self.lmean:

            if abs(self.rfmean-self.rbmean) < 0.01:
                msg.angular.z = 0.0
            else:
                msg.angular.z = (float) (self.rfmean - self.rbmean)

        self.vel_pub.publish(msg)

    def process_scan(self, data):        
        self.distances = np.array(data.ranges)
        self.angles = np.array(range(361))

        focus_area = np.where((self.distances > 0) & (self.distances < 2))
        self.distances = self.distances[focus_area]
        self.angles = self.angles[focus_area]

        left = np.where(
            (self.angles > 70) & (self.angles < 110)
        )
        right = np.where(
            (self.angles > 250) & (self.angles < 290)
        )
        front = np.where(
            (self.angles < 20) | (self.angles > 340)
        )

        left_front = np.where(
            (self.angles > 60) & (self.angles < 90)
        )
        left_back = np.where(
            (self.angles > 90) & (self.angles < 120)
        )

        right_front = np.where(
            (self.angles > 240) & (self.angles < 270)
        )
        right_back = np.where(
            (self.angles > 270) & (self.angles < 300)
        )

        if len(self.distances[front]) <= 0:
            self.fmean = 100 
        else:
            self.fmean = sum(self.distances[front]) / len(self.distances[front])
        
        if len(self.distances[left]) <= 0:
            self.lmean = 100 
        else:
            self.lmean = sum(self.distances[left]) / len(self.distances[left])

        if len(self.distances[left_front]) <= 0:
            self.lfmean = 100 
        else:
            self.lfmean = sum(self.distances[left_front]) / len(self.distances[left_front])

        if len(self.distances[left_back]) <= 0:
            self.lbmean = 100 
        else:
            self.lbmean = sum(self.distances[left_back]) / len(self.distances[left_back])

        if len(self.distances[right]) <= 0:
            self.rmean = 100 
        else:
            self.rmean = sum(self.distances[right]) / len(self.distances[right])

        if len(self.distances[right_front]) <= 0:
            self.rfmean = 100 
        else:
            self.rfmean = sum(self.distances[right_front]) / len(self.distances[right_front])

        if len(self.distances[right_back]) <= 0:
            self.rbmean = 100 
        else:
            self.rbmean = sum(self.distances[right_back]) / len(self.distances[right_back])

        # print(f"distances: {len(self.distances)} ")
        # print(f"angles: {len(self.angles)} ")
        # print(f"left: {self.left[0]} ")
        # print(f"right: {len(self.right)} ")

def main(args=None):
    rclpy.init(args=args)
    node = wall_follower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
