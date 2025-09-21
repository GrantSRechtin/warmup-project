import rclpy
from rclpy.node import Node
import numpy as np

from time import sleep
from math import pi

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class PersonFollowerNode(Node):
    def __init__(self):
        super().__init__('person_follower_node')

        self.active = False
        self.target_angle = 0
        self.scans
        self.degs_to_rads = np.pi / 180
        self.distances = np.array()
        self.angles = np.array()
        
        self.create_timer(0.1, self.run_loop)
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.create_subscription(String, 'state', 10)

        self.completion_pub = self.create_publisher(bool, 'full_empty', 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def run_loop(self):
        if self.active and len(self.scans) > 0:
            # compute cluster centroid
            x_coordinates = np.multiply(self.scans, np.cos(np.multiply(self.angles, self.degs_to_rads)))
            y_coordinates = np.multiply(self.scans, np.sin(np.multiply(self.angles, self.degs_to_rads)))

            self.avg_x = np.average(x_coordinates)
            self.avg_y = np.average(y_coordinates)

            self.item_distance = np.sqrt(self.avg_x**2 + self.avg_y**2)
            theta = np.arctan2(self.avg_y, self.avg_x)   # error angle in radians
            self.item_error = theta

            # --- make Twist message ---
            msg = Twist()

            # simple proportional controller
            linear_speed = 0.2 * (self.item_distance - 0.5)   # keep ~0.5m away
            angular_speed = 1.0 * self.item_error            # turn to face target

            # safety limits
            msg.linear.x = max(min(linear_speed, 0.3), 0.0)   # clamp forward speed [0, 0.3]
            msg.angular.z = max(min(angular_speed, 1.0), -1.0) # clamp angular speed [-1, 1]

            # publish Twist
            self.vel_pub.publish(msg)

        else:
            # stop if not active or no target
            stop_msg = Twist()
            self.vel_pub.publish(stop_msg)


    def find_target_angle(self):

        max_sum = 0
        self.target_angle = 0

        for i in range(len(self.distances)-44):
            grouping = self.distances[i:i+45]
            max_sum = max(sum(grouping), max_sum)
            self.target_angle = self.angles[i+22] if sum(grouping) > max_sum else self.target_angle

        self.target_angle = self.target_angle if self.target_angle <= 180 else (self.target_angle-360)

    def check_follow_people(self):
        """Uses scan data to determine if there is a followable object.
        If a followable object, updates self.scans and self.angles to the cluster,
        and returns True. Otherwise returns False."""

        ang = self.angles
        dist = self.distances

        # find closest object
        closest_idx = np.argmin(dist)
        closest_angle = ang[closest_idx]
        closest_dist = dist[closest_idx]

        # cluster = all points within ~0.1m of closest distance
        cluster_indices = np.where(np.abs(dist - closest_dist) < 0.1)[0]

        # angles of the cluster
        cluster_angles = ang[cluster_indices]
        cluster_dists = dist[cluster_indices]

        # check if cluster is compact enough to be a "person"
        if (len(cluster_indices) > 0 and
            (np.max(cluster_angles) - np.min(cluster_angles) <= 45
            or ((np.max(cluster_angles) >= 315) and (np.min(cluster_angles) <= 45)))):
            
            # update the stored scans + angles for run_loop()
            self.scans = cluster_dists
            self.angles = cluster_angles

            return True
        else:
            return False


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