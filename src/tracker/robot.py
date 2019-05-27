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

SAMPLES = 10
LINEAR_SAMPLES = 5
SAFE_DISTANCE = 0.7
TURN_THRESH = 0.35
LINEAR_SPEED = 0.2
SLOW_LINEAR_SPEED = 0.1
ANGULAR_SPEED = 0.7
WIFI_SIGNAL_GOAL = -23

class Robot:
    def __init__(self, freq=1):

        rospy.Subscriber("/leader/wifi_signal", WifiStrength, self.wifi_callback)
        rospy.Subscriber("/leader/base_pose_ground_truth", Odometry, self.self_odometry_callback)
        rospy.Subscriber("/leader/base_scan", LaserScan, self.laser_scan_callback)


        self.frequency = freq
        self.is_moving = True
        self.found_goal = False

        # for finding the average wifi strength over SAMPLES samples
        self.samples = 0
        self.tempStrength = 0

        self.prevWifiStrength = None
        self.currWifiStrength = None
        #self.avgWifiStrength = None

        # position variables
        self.x = None
        self.y = None
        self.yaw = None

        self.cmd_msg = Twist()

        self.xArr = []
        self.yArr = []
        self.signalArr = []

        self.cmd_pub = rospy.Publisher("/leader/cmd_vel", Twist, queue_size = 1)

        self.curr_state = "FORWARDS"
        self.next_state = None


    def spin(self):
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            print(self.curr_state)
            if self.is_moving == False or self.found_goal == True:
                self.cmd_msg = Twist()

            else:

                # drive straight towards wifi
                if self.curr_state == "FORWARDS":
                    self.cmd_msg = Twist()
                    self.cmd_msg.linear.x = LINEAR_SPEED


                elif self.curr_state == "ROTATE":
                    self.cmd_msg = Twist()
                    self.cmd_msg.angular.z = ANGULAR_SPEED
                    self.cmd_msg.linear.x = SLOW_LINEAR_SPEED


            self.cmd_pub.publish(self.cmd_msg)
            r.sleep()
            self.curr_state = self.next_state



    def wifi_callback(self, msg):
        # if self.samples < SAMPLES:

        #     self.tempStrength += msg.wifi_strength
        #     self.samples += 1

        # else:
        #     # store average wifi strength
        #     self.avgWifiStrength = self.tempStrength / self.samples

        #     # reset values to take new samples for next average
        #     self.tempStrength = 0
        #     self.samples = 0

        self.prevWifiStrength = self.currWifiStrength
        self.currWifiStrength = msg.wifi_strength

        print("prev: " + str(self.prevWifiStrength) + "| curr: " + str(self.currWifiStrength))

        if(self.currWifiStrength >= WIFI_SIGNAL_GOAL):
            self.found_goal = True
            rospy.loginfo("The turtlebot has reached the goal wifi signal!")
        else:

            if abs(abs(self.prevWifiStrength) - abs(self.currWifiStrength)) <= TURN_THRESH:
                self.next_state = "ROTATE"

            elif abs(self.prevWifiStrength) > abs(self.currWifiStrength):
                self.next_state = "FORWARDS"

        rospy.loginfo("current strength: " + str(self.currWifiStrength))


    def self_odometry_callback(self, odometry_msg):
        pose = odometry_msg.pose.pose
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]

        # update invidual robot's position
        self.x = pose.position.x
        self.y = pose.position.y
        self.yaw = yaw


    # laser scan for obstacle avoidance
    def laser_scan_callback(self, laser_scan_msg):
        ranges = numpy.array(laser_scan_msg.ranges)
        min_laser_scan_reading = ranges.min()
        reading_string = "Minimum laser scan reading: {}".format(min_laser_scan_reading)
        rospy.logdebug(reading_string) 


        # Assumption! Just stop once and exit.
        if min_laser_scan_reading < SAFE_DISTANCE:
            #rospy.loginfo("is moving? " + str(self.is_moving))
            self.is_moving = False
        else:
            self.is_moving = True



    def shutdown(self):
        rospy.loginfo("Shutting down the turtlebot")
        self.cmd_pub.publish(Twist())
        rospy.sleep(1)
        rospy.loginfo("Goodbye!")