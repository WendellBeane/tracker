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

        # initialize subscriber for static target (this code assumes that target is known)
        rospy.Subscriber("/robot_0/base_pose_ground_truth", Odometry, self.odometry_callback_target)

        # target pose variables
        self.x_target = None
        self.y_target = None
        # self.yaw_target = None    # TODO: might want to account for this later when determining goal points for each robot

        # initialize subscribers (for hunters)
        rospy.Subscriber("/robot_1/base_pose_ground_truth", Odometry, self.odometry_callback_1)
        rospy.Subscriber("/robot_1/base_scan", LaserScan, self.laser_scan_callback_1)

        # pose variables
        self.x = None
        self.y = None
        self.yaw = None

        # behavioral state machine
        self.curr_state = "COMPUTE_PTS"
        self.next_state = None


    def spin(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.loginfo("X: " + str(self.x) + " Y: " + str(self.y) + " YAW: " + str(self.yaw))
            rospy.loginfo("target_X: " + str(self.x_target) + " target_Y: " + str(self.y_target))


            r.sleep()


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