#!/usr/bin/env python
import rospy
import tf
from math import sin, cos, pi

# import the custom formation message
from tracker.msg import Formation
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry


DISTANCE = 0.9
class Compute:

    def __init__ (self):

        # subscribe to the target location
        rospy.Subscriber('/robot_0/base_pose_ground_truth', Odometry, self.target_odom_callback)

        self.target_x = None
        self.target_y = None
        self.target_yaw = None

        self.formation_pub = rospy.Publisher('compute', Formation, queue_size=10)


    # simulates the case where the targets odom data is known
    def target_odom_callback(self, msg):
        pose = msg.pose.pose
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]

        # update target's position
        self.target_x = pose.position.x
        self.target_y = pose.position.y
        self.target_yaw = yaw


if __name__ == "__main__":
    rospy.init_node('compute', anonymous=True)
    compute = Compute()
    r = rospy.Rate(2)

    while not rospy.is_shutdown():
        # get a new wifi signal message
        msg = Formation()
        msg.stamp = rospy.Time.now()
        msg.frame_id = "target"

        point0 = Point()
        point1 = Point()
        point2 = Point()

        # based on triangular formation computations, rotates based on target's yaw
        if compute.target_x and compute.target_y and compute.target_yaw:
            point0.x = compute.target_x + DISTANCE * cos(compute.target_yaw + float(pi)/float(6.0))
            point0.y = compute.target_y + DISTANCE * sin(compute.target_yaw + float(pi)/float(6.0))
            
            point1.x = compute.target_x + DISTANCE * cos(compute.target_yaw + float(5.0*pi)/float(6.0))
            point1.y = compute.target_y + DISTANCE * sin(compute.target_yaw + float(5.0*pi)/float(6.0))

            point2.x = compute.target_x + DISTANCE * cos(compute.target_yaw + float(3.0*pi)/float(2.0))
            point2.y = compute.target_y + DISTANCE * sin(compute.target_yaw + float(3.0*pi)/float(2.0))

        # Note: The formation points have not been computed if all points are (0,0)
        msg.points.append(point0)
        msg.points.append(point1)
        msg.points.append(point2)


        # publish the message
        compute.formation_pub.publish(msg)
        r.sleep()
