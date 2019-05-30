from numpy.random import rand
import numpy 
from math import pow, sqrt, atan2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rospy
import tf

from tracker.msg import Formation
from sensor_msgs.msg import LaserScan

# constants
GOAL_THRESH = 0.275
SAFE_DISTANCE = 0.75

class Robot:
    def __init__(self, name, freq=1):

        self.name = name

        # initialize subscribers (for hunters)
        rospy.Subscriber("/" + self.name + "/base_pose_ground_truth", Odometry, self.odometry_callback_1)
        rospy.Subscriber("/" + self.name + "/base_scan", LaserScan, self.laser_scan_callback_1)
        rospy.Subscriber("/robot_0/base_pose_ground_truth", Odometry, self.target_odom_callback)
        rospy.Subscriber("/compute", Formation, self.compute_callback)
        
        # publishers for hunters
        self.cmd_pub = rospy.Publisher("/" + self.name + "/cmd_vel", Twist, queue_size = 1)
        self.cmd_vel = Twist()

        self.is_moving = True
        self.reached_goal = False

        # pose variables
        self.x = None
        self.y = None
        self.yaw = None
        self.target_x = None
        self.target_y = None
        self.target_yaw = None
        self.goals = None
        self.x_goal = None
        self.y_goal = None 
        # behavioral state machine
        self.curr_state = "COMPUTE" #TODO: Change to SEARCH or IDLE when that is written
        self.next_state = None


    def spin(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():

            # obstacle detected

            if self.reached_goal == True:
                self.cmd_vel = Twist()

            elif self.is_moving == False:
                self.cmd_vel = Twist()

                self.cmd_vel.angular.z = 0.5

            else:

                # behavior state machine is secondary to obstacle handling
                if self.curr_state == "COMPUTE":
                    self.x_goal = None
                    self.y_goal = None
                    rospy.loginfo("Current state is compute")

                    # target has not yet been detected
                    if self.goals is None:
                        self.next_state = "COMPUTE"

                    # target has been detected
                    else:

                        self.reached_goal == False
                        # assign goals based on robot name 
                        if self.name == "robot_1":
                            self.x_goal, self.y_goal = self.goals[0].x, self.goals[0].y
                        elif self.name == "robot_2":
                            self.x_goal, self.y_goal = self.goals[1].x, self.goals[1].y
                        elif self.name == "robot_3":
                            self.x_goal, self.y_goal = self.goals[2].x, self.goals[2].y

                        # go to next state only if goals have been assigned to robot
                        if self.x_goal != None and self.y_goal != None:
                            self.next_state = "APPROACH"
                        else:
                            self.next_state = "COMPUTE"

                # approach formation points using the goto location ROS tutorial
                elif self.curr_state == "APPROACH":   

                    # send velocity commands if the goal is outside of certain error threshold
                    if self.euclidean_distance(self.x_goal, self.y_goal) >= GOAL_THRESH:
                        self.cmd_vel = Twist()
                        self.cmd_vel.linear.x = self.linear_vel(self.x_goal, self.y_goal)
                        self.cmd_vel.angular.z = self.angular_vel(self.x_goal, self.y_goal)
                        self.next_state = "APPROACH"

                    else:
                        # stop movement once goal reached
                        self.reached_goal == True
                        self.cmd_vel = Twist()
                        self.next_state = "ALIGN"

                # TODO: rotate robot until the heading matches the target's yaw
                elif self.curr_state == "ALIGN":
                    print("In ALIGN state")
                    self.align_with_target()


            self.cmd_pub.publish(self.cmd_vel)
            self.curr_state = self.next_state
            r.sleep()

    def align_with_target(self):
        if abs(self.yaw - self.target_yaw) >= 1:
            self.cmd_vel= Twist()
            self.cmd_vel.angular.z = (self.yaw - self.target_yaw)
            

        if abs(self.yaw - self.target_yaw) <= 1: 
            rospy.loginfo("aligned with target")
            self.cmd_vel= Twist()
            self.cmd_vel.angular.z = 0
            self.curr_state = "COMPUTE"

    # The functions below are from the following ROS tutorial: http://wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal
    def euclidean_distance(self, x_goal, y_goal):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((x_goal - self.x), 2) + pow((y_goal - self.y), 2))

    def linear_vel(self, x_goal, y_goal, constant=1.5):
        return constant * self.euclidean_distance(x_goal, y_goal)
    
    def steering_angle(self, x_goal, y_goal):
        return atan2(y_goal - self.y, x_goal - self.x)

    def angular_vel(self, x_goal, y_goal, constant=6):

        return constant * (self.steering_angle(x_goal, y_goal) - self.yaw)

    def compute_callback(self, formation_msg):
        """Callback function to get 3 formation goal points."""
        goals = formation_msg.points

        # update goals
        if goals[0] == goals[1] and goals[1] == goals[2] and goals[0] == goals[2]:
            self.goals = None
        else:
            if goals != self.goals:
                self.goals = goals
                self.curr_state = "COMPUTE"
            
    def target_odom_callback(self, odometry_msg):
        pose = odometry_msg.pose.pose
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]

        # update invidual robot's position
        self.target_x = pose.position.x
        self.target_y = pose.position.y
        self.target_yaw = yaw


    def odometry_callback_1(self, odometry_msg):
        """Callback function for hunter odom data."""
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


    # TODO: edit this so we only look forward facing and can also navigate around the obstacle
    # laser scan for obstacle avoidance
    def laser_scan_callback_1(self, laser_scan_msg):
        """Callback function for hunter lidar data."""
        ranges = numpy.array(laser_scan_msg.ranges)
        min_laser_scan_reading = numpy.min(ranges[numpy.nonzero(ranges)])
        reading_string = "Minimum laser scan reading: {}".format(min_laser_scan_reading)
        rospy.logdebug(reading_string) 

        # obstacle handling
        if min_laser_scan_reading < SAFE_DISTANCE:
            self.is_moving = False
        else:
            self.is_moving = True



    def shutdown(self):
        """Clean shutdown of node."""
        rospy.loginfo("Shutting down the turtlebot")
        self.cmd_pub.publish(Twist())
        rospy.sleep(1)
        rospy.loginfo("Goodbye!")