"""Teleoperation node to control a neato with the user's keyboard inputs.
    Contains E-stop functionality via multithreading.
"""

#Teleop libraries
import tty
import select
import sys
import termios

#Estop libraries
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math

#Multithreading Libraries
from rclpy.node import Node
from threading import Thread, Event
from time import sleep

#Standard libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class TeleopNode(Node):
    """Teleoperation node to control a neato with the user's keyboard inputs.
        WS controls forward and backward, AD rotate the robot in place.
    """

    def __init__(self):
        """Initializes the class.
        """
        super().__init__("teleop_estop")
        self.e_stop = Event()
        # create a thread to handle estop component
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Bool, 'estop', self.handle_estop, 10)
        self.run_loop_thread = Thread(target=self.run_loop)
        self.run_loop_thread.start()

        #Timer for motors
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)

        #Create the publisher for cmd_vel that tells the motors to move.
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)

        # Define getKey() settings
        self.settings = termios.tcgetattr(sys.stdin)


    def run_loop(self):
        """Key commands for teleoperation robot control
        """

        key = self.getKey()
        if not self.e_stop.is_set():
            vel = Twist()
            if key == "w":
                vel.linear.x = 0.3
                vel.angular.z = 0.0
            elif key == "s":
                vel.linear.x = -0.3
                vel.angular.z = 0.0
            elif key == "a":
                vel.linear.x = 0.0
                vel.angular.z = 0.3
            elif key == "d":
                vel.linear.x = 0.0
                vel.angular.z = -0.3
            elif key == None:
                vel.linear.x = 0.0
                vel.angular.z = 0.0
            elif key == "/x03":
                quit()

        self.publisher.publish(vel)
        key = None


    def getKey(self):
        """Finds current pressed key

        Returns
            String key representing the keyboard key that the user is pressing.
        """
        if not self.e_stop.is_set():
            tty.setraw(sys.stdin.fileno())
            select.select([sys.stdin], [], [], 0)
            key = sys.stdin.read(1)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            return key
    
def handle_estop(self, msg):
        """Handles estop messages.

        Args:
            msg (std_msgs.msg.Bool): true if estopped
        """ 
        if msg.data:
            self.e_stop.set()
            self.drive(linear=0.0, angular=0.0)


def main(args=None):
    """Initialize, run, cleanup.
    """
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

