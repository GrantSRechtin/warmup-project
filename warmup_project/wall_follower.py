import rclpy
from rclpy.node import Node
import numpy as np

from math import pi
from time import sleep

from neato2_interfaces.msg import Bump
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class wall_follower(Node):
    def __init__(self):
        super().__init__('wall_follower')

        self.lin_vel = 0.1
        self.ang_vel = 2

        self.flmean = 0.0
        self.frmean = 0.0

        self.lmean = 0.0
        self.lfmean = 0.0
        self.lbmean = 0.0

        self.rmean = 0.0
        self.rfmean = 0.0
        self.rbmean = 0.0

        self.prev_f = self.flmean

        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.create_timer(0.1, self.run_loop)
    
    def run_loop(self):
        msg = Twist()

        msg.linear.x = 0.1

        if self.flmean != self.prev_f:
            if (self.flmean < self.lmean and self.flmean < self.rmean) or (self.frmean < self.lmean and self.frmean < self.rmean):
                print('forward')

                if self.lmean < self.rmean:
                    msg.angular.z = -pi / 10
                else:
                    msg.angular.z = pi / 10

                msg.linear.x = 0.0

                self.vel_pub.publish(msg)
                sleep(5)

            elif self.lmean < self.rmean:

                if abs(self.lfmean-self.lbmean) < 0.01 or abs(self.lfmean-self.lbmean) > 0.3:
                    msg.angular.z = 0.0
                else:
                    msg.angular.z = (float) (self.lfmean - self.lbmean) * self.ang_vel

                self.vel_pub.publish(msg)    

            else:

                if abs(self.rfmean-self.rbmean) < 0.01 or abs(self.rfmean-self.rbmean) > 0.3:
                    msg.angular.z = 0.0
                else:
                    msg.angular.z = (float) (self.rfmean - self.rbmean) * self.ang_vel

                self.vel_pub.publish(msg)

            print(f"flmean: {self.flmean}")
            print(f"frmean: {self.frmean}")
            print(f"lmean: {self.lmean}")
            print(f"rmean: {self.rmean}")
            print(f"lin vel: {msg.linear.x}")
            print(f"ang vel: {msg.angular.z}")

            self.prev_f = self.flmean

    def process_scan(self, data):  

        self.distances = np.array(data.ranges)
        self.angles = np.array(range(361))

        focus_area = np.where((self.distances > 0.1) & (self.distances < 2))
        self.distances = self.distances[focus_area]
        self.angles = self.angles[focus_area]

        front_left = np.where(
            (self.angles < 30) | (self.angles > 350)
        )
        front_right = np.where(
            (self.angles > 330) | (self.angles < 10)
        )

        left = np.where(
            (self.angles > 70) & (self.angles < 110)
        )
        right = np.where(
            (self.angles > 250) & (self.angles < 290)
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

        self.flmean = 100 if len(self.distances[front_left]) <= 0 else sum(self.distances[front_left]) / len(self.distances[front_left])
        self.frmean = 100 if len(self.distances[front_right]) <= 0 else sum(self.distances[front_right]) / len(self.distances[front_right])

        self.lmean = 100 if len(self.distances[left]) <= 0 else sum(self.distances[left]) / len(self.distances[left])
        self.lfmean = 100 if len(self.distances[left_front]) <= 0 else sum(self.distances[left_front]) / len(self.distances[left_front])
        self.lbmean = 100 if len(self.distances[left_back]) <= 0 else sum(self.distances[left_back]) / len(self.distances[left_back])

        self.rmean = 100 if len(self.distances[right]) <= 0 else sum(self.distances[right]) / len(self.distances[right])
        self.rfmean = 100 if len(self.distances[right_front]) <= 0 else sum(self.distances[right_front]) / len(self.distances[right_front])
        self.rbmean = 100 if len(self.distances[right_back]) <= 0 else sum(self.distances[right_back]) / len(self.distances[right_back])

def main(args=None):
    rclpy.init(args=args)
    node = wall_follower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
