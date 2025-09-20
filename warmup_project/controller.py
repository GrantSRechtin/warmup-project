import rclpy
from rclpy.node import Node
import numpy as np

from math import pi

from neato2_interfaces.msg import Bump
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class controller(Node):
    def __init__(self):
        super().__init__('controller')
        
        self.distances = np.array()
        self.angles = np.array()

        self.state = 'Spiral'
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10) #for detecting followable obj
        self.create_subscription(Bump, 'bump', self.process_bump, 10)
        self.create_subscription(bool, 'full_empty', self.process_full_empty, 10)
        self.publisher_ = self.create_publisher(String, 'state', 10)

        self.create_timer(0.1, self.run_loop)
        self.full_empty = True
    
    def run_loop(self):
        msg = String()

        #need some kind of logic to check if any of the bools have changes. that is when we should re-evalueate state
        #this is because we want the state message to be an event that starts the state from the BEGINNING
        if self.state == 'Person Following' and self.process_bump():
            msg = 'Turn Around'
        elif self.state == 'Person Following' and self.full_empty and not self.check_follow_people(): #and prob another thing that lets us know that its even time to switch states
            #if PF and tons of stuff
            msg = 'Turn Around'
        elif self.state == 'Person Following' and not self.full_empty and not self.check_follow_people():
            #if PF and tons of stuff
            msg = 'Spiral'
        elif self.state == 'Turn Around' and self.process_bump():
            msg = 'Turn Around' #this should start a new turn around
        elif self.state == 'Turn Around' and not self.check_follow_people(): #also check if turn around is complete        Turn complete bool
            msg = 'Spiral'
        elif self.state == 'Turn Around' and self.check_follow_people(): #also check if turn around is complete            Turn complete bool
            msg = 'Person Following'
        #POTENTIALLY MAKE LOGIC FOR IF NO CLEAR FOLLOW OBJECT AT THE END OF TURN AROUND
        elif self.state == 'Spiral' and self.check_follow_people():
            msg = 'Person Following'
        elif self.state == 'Spiral' and self.process_bump():
            msg = 'Turn Around'

        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)

    def process_scan(self, data):
        #gets locations of objects

        self.distances = np.array(data.ranges)
        self.angles = np.array(range(361))

        focus_area = np.where((self.distances > 0.1) & (self.distances < 2))
        self.distances = self.distances[focus_area]
        self.angles = self.angles[focus_area]
    
    def check_follow_people(self):
        """uses scan data to determine if there is a followable object
            If a followable object, returns true"""
        
        ang = self.angles
        dist = self.distances

        closest = ang[np.min(dist)]
        dist_range = np.where((dist < closest+0.1))

        not_front_range = np.where((ang > 45) & (ang < 315))
        not_front = dist[not_front_range]

        if (abs(np.max(ang[dist_range]) - np.min(ang[dist_range])) <= 45) or ((np.max(ang[dist_range]) >= 315) & (np.min(ang[dist_range]) <= 45) & len(not_front) == 0):
            return True
        else:
            return False

    def process_bump(self, msg):
            """Callback for handling a bump sensor input."
            Input: 
                msg (Bump): a Bump type message from the subscriber.
            """
            # Set the bump state to True if any part of the sensor is pressed
            return (msg.left_front == 1 or \
                    msg.right_front == 1 or \
                    msg.left_side == 1 or \
                    msg.right_side == 1)

    def process_full_empty(self,msg):
        self.full_empty = msg.data

def main(args=None):
    rclpy.init(args=args)
    node = controller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
