import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class TurnAroundNode(Node):
    def __init__(self):
        super().__init__('turn_around_node')
        
        self.create_timer(0.1, self.run_loop)
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)

        self.completion_pub = self.create_publisher(bool, 'turn_complete', 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    
    def run_loop(self):
        msg = Twist()



        self.vel_pub.publish(msg)

    def process_scan(self, msg):
        if msg.ranges[0] != 0.0:
            self.distance_to_obstacle = msg.ranges[0]


def main(args=None):
    rclpy.init(args=args)
    node = TurnAroundNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()