from numpy.random import rand
import numpy 
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
#from std_srvs.srv import TriggerResponse
from nav_msgs.msg import Odometry
import rospy
import tf

from capture_prey_sim.msg import WifiStrength
from sensor_msgs.msg import LaserScan


class Robot:
    def __init__(self, freq=1):
        print("in the init method")


    def spin(self):
        print("in spin!")




    # def shutdown(self):
    #     rospy.loginfo("Shutting down the turtlebot")
    #     self.cmd_pub.publish(Twist())
    #     rospy.sleep(1)
    #     rospy.loginfo("Goodbye!")