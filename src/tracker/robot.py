from numpy.random import rand
import numpy 
from math import pow, sqrt, atan2
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
#from std_srvs.srv import TriggerResponse
from nav_msgs.msg import Odometry
import rospy
import tf

from tracker.msg import Formation
from sensor_msgs.msg import LaserScan

# constants
GOAL_THRESH = 0.15

class Robot:
    def __init__(self, name, freq=1):

        self.name = name

        # initialize subscribers (for hunters)
        rospy.Subscriber("/" + self.name + "/base_pose_ground_truth", Odometry, self.odometry_callback_1)
        rospy.Subscriber("/" + self.name + "/base_scan", LaserScan, self.laser_scan_callback_1)

        rospy.Subscriber("/compute", Formation, self.compute_callback)

        # publishers for hunters
        self.cmd_pub = rospy.Publisher("/" + self.name + "/cmd_vel", Twist, queue_size = 1)
        self.cmd_vel = Twist()

        # pose variables
        self.x = None
        self.y = None
        self.yaw = None

        self.goals = None

        # behavioral state machine
        self.curr_state = "COMPUTE" #TODO: Change to SEARCH or IDLE when that is written
        self.next_state = None


    def spin(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():

            if self.curr_state == "COMPUTE":
                x_goal = None
                y_goal = None

                if self.goals is None:  # wait for the target to be detected and pose to be stored
                    self.next_state = "COMPUTE"
                else:
                    if self.name == "robot_1":
                        x_goal, y_goal = self.goals[0].x, self.goals[0].y
                    elif self.name == "robot_2":
                        x_goal, y_goal = self.goals[1].x, self.goals[1].y
                    elif self.name == "robot_3":
                        x_goal, y_goal = self.goals[2].x, self.goals[2].y

                    if x_goal != None and y_goal != None:
                        self.next_state = "APPROACH"
                    else:
                        self.next_state = "COMPUTE"



            elif self.curr_state == "APPROACH":   
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


    def compute_callback(self, formation_msg):
        goals = formation_msg.points

        # update goals
        if goals[0] == goals[1] and goals[1] == goals[2] and goals[0] == goals[2]:
            self.goals = None
        else:
            self.goals = goals


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


    def shutdown(self):
        rospy.loginfo("Shutting down the turtlebot")
        self.cmd_pub.publish(Twist())
        rospy.sleep(1)
        rospy.loginfo("Goodbye!")