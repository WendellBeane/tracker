import numpy as np 
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
import rospy
import tf
from sensor_msgs.msg import LaserScan


class Control_Robot:
	def __init__(self, freq = 1):

		rospy.Subscriber("/robot_1/base_pose_ground_truth", Odometry, self.odometry_callback_1)
        rospy.Subscriber("/robot_1/base_scan", LaserScan, self.laser_scan_callback_1)

        target_pose = rospy.wait_for_message("/target/base_pose_ground_truth", Odometry)
        rospy.Subscriber("/target/base_pose_ground_truth", Odometry, self.target_subscriber)

        self.x = None
        self.y = None
        self.yaw = None
        #States are tracking, collision avoidance, cooperative collision avoidance, circle formation
        self.curr_state = "TRACKING"

        self.prey_x 
        self.prey_y
        self.prey_yaw

        self.cmd_pub = rospy.Publisher("/robot_1/cmd_vel", Twist, queue_size = 1)
        self.cmd_vel = Twist()


    def spin(self):
    	r = rospy.Rate(10)
        while not rospy.is_shutdown():
        	if self.curr_state == "TRACKING":
        		self.cmd_vel = self.set_tracking_velocity()

        	if self.curr_state == "AVOID_OBSTACLE":
        		self.avoid_obstacle()

        	if self.curr_state == "CIRCLE_TARGET":
        		self.circle_target()


       		self.cmd_pub.publish(self.cmd_vel)

    def odometry_callback(self, msg):
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

    def target_subscriber(self, msg);
    	pose = odometry_msg.pose.pose
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]

        # update invidual robot's position
        self.prey_x = pose.position.x
        self.prey_y = pose.position.y
        self.prey_yaw = yaw
        self.prey_lin_velocity = msg.twist.twist.linear
        self.prey_angular_velocity = msg.twist.twist.angular




