from numpy.random import rand
import numpy 
from math import pow, sqrt, atan2
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
#from std_srvs.srv import TriggerResponse
from nav_msgs.msg import Odometry
import rospy
import tf

from capture_prey_sim.msg import WifiStrength
from sensor_msgs.msg import LaserScan

# constants
GOAL_THRESH = 0.15

class Robot:
    def __init__(self, freq=1):

        # initialize subscriber for static target (this code assumes that target is known)
        rospy.Subscriber("/robot_0/base_pose_ground_truth", Odometry, self.odometry_callback_target)

        # target pose variables
        self.x_target = None
        self.y_target = None
        # self.yaw_target = None    # TODO: might want to account for this later when determining goal points for each robot

        # initialize subscribers (for hunters)
        rospy.Subscriber("/robot_1/base_pose_ground_truth", Odometry, self.odometry_callback_1)
        rospy.Subscriber("/robot_1/base_scan", LaserScan, self.laser_scan_callback_1)

        # publishers for hunters
        self.cmd_pub = rospy.Publisher("/robot_1/cmd_vel", Twist, queue_size = 1)
        self.cmd_vel = Twist()

        # pose variables
        self.x = None
        self.y = None
        self.yaw = None

        # behavioral state machine
        self.curr_state = "COMPUTE" #TODO: Change to SEARCH or IDLE when that is written
        self.next_state = None


    def spin(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            # rospy.loginfo("X: " + str(self.x) + " Y: " + str(self.y) + " YAW: " + str(self.yaw))
            # rospy.loginfo("target_X: " + str(self.x_target) + " target_Y: " + str(self.y_target))

            if self.curr_state == "COMPUTE":
                print("in compute")
                # TODO: Publish list of 3 formation points, each hunter grabs a point 
                self.next_state = "APPROACH"

            elif self.curr_state == "APPROACH":
                rospy.loginfo("CURRENT STATE: APPROACH")

                x_goal, y_goal = self.x_target, self.y_target + 1 # TODO: computations needed here
                
                # send commands to hunter bot if the goal is outside of certain error threshold
                if self.euclidean_distance(x_goal, y_goal) >= GOAL_THRESH:
                    self.cmd_vel = Twist()
                    self.cmd_vel.linear.x = self.linear_vel(x_goal, y_goal)
                    self.cmd_vel.angular.z = self.angular_vel(x_goal, y_goal)
                    self.next_state = "APPROACH"
                else:
                    # stop movement
                    self.cmd_vel = Twist()
                    self.next_state = "ALIGN"

            elif self.curr_state == "ALIGN":
                # TODO: rotate robot until the heading matches the target's yaw
                print("In ALIGN state")

            self.cmd_pub.publish(self.cmd_vel)
            r.sleep()
            self.curr_state = self.next_state


    # The functions below are from the following ROS tutorial: http://wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal
    def euclidean_distance(self, x_goal, y_goal):
        print("GOAL: " + str(x_goal) +", " + str(y_goal))
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((x_goal - self.x), 2) + pow((y_goal - self.y), 2))

    def linear_vel(self, x_goal, y_goal, constant=1.5):
        return constant * self.euclidean_distance(x_goal, y_goal)
    
    def steering_angle(self, x_goal, y_goal):
        return atan2(y_goal - self.y, x_goal - self.x)

    def angular_vel(self, x_goal, y_goal, constant=6):
        return constant * (self.steering_angle(x_goal, y_goal) - self.yaw)



    def odometry_callback_target(self, odometry_msg):
        pose = odometry_msg.pose.pose
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]

        # update invidual robot's position
        self.x_target = pose.position.x
        self.y_target = pose.position.y
        self.yaw_target = yaw


    def odometry_callback_1(self, odometry_msg):
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



    def laser_scan_callback_1(self, laser_scan_msg):
        ranges = numpy.array(laser_scan_msg.ranges)
        min_laser_scan_reading = ranges.min()
        reading_string = "Minimum laser scan reading: {}".format(min_laser_scan_reading)
        rospy.logdebug(reading_string) 


        # # Assumption! Just stop once and exit.
        # if min_laser_scan_reading < SAFE_DISTANCE:
        #     #rospy.loginfo("is moving? " + str(self.is_moving))
        #     self.is_moving = False
        # else:
        #     self.is_moving = True


    # def shutdown(self):
    #     rospy.loginfo("Shutting down the turtlebot")
    #     self.cmd_pub.publish(Twist())
    #     rospy.sleep(1)
    #     rospy.loginfo("Goodbye!")