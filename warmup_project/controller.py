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
             msg = 'TurnAround'
        elif self.state == 'Person Following' and self.full_empty: #and prob another thing that lets us know that its even time to switch states
            #if PF and tons of stuff
            msg = 'TurnAround'
        elif self.state == 'Person Following' and not self.full_empty:
            #if PF and tons of stuff
            msg = 'Spiral'
        elif self.state == 'TurnAround' and self.process_bump():
            msg = 'TurnAround' #this should start a new turn around
        elif self.state == 'TurnAround' and not self.check_follow_people(): #also check if turn around is complete
            msg = 'Spiral'
        elif self.state == 'TurnAround' and self.check_follow_people(): #also check if turn around is complete
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
        # taken from wall_follower        
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

        self.fmean = 100 if len(self.distances[front]) <= 0 else sum(self.distances[front]) / len(self.distances[front])

        self.lmean = 100 if len(self.distances[left]) <= 0 else sum(self.distances[left]) / len(self.distances[left])
        self.lfmean = 100 if len(self.distances[left_front]) <= 0 else sum(self.distances[left_front]) / len(self.distances[left_front])
        self.lbmean = 100 if len(self.distances[left_back]) <= 0 else sum(self.distances[left_back]) / len(self.distances[left_back])

        self.rmean = 100 if len(self.distances[right]) <= 0 else sum(self.distances[right]) / len(self.distances[right])
        self.rfmean = 100 if len(self.distances[right_front]) <= 0 else sum(self.distances[right_front]) / len(self.distances[right_front])
        self.rbmean = 100 if len(self.distances[right_back]) <= 0 else sum(self.distances[right_back]) / len(self.distances[right_back])
    
def check_follow_people(self):
    """uses scan data to determine if there is a followable object
        If a followable object, returns true"""

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
    if msg.data:
        full_empty = True
    else:
        full_empty = False

def main(args=None):
    rclpy.init(args=args)
    node = controller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
