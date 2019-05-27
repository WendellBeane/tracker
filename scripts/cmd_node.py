#!/usr/bin/python
from tracker.robot import Robot
import rospy

if __name__ == "__main__":
    rospy.init_node("cmd")
    robot = Robot()
    robot.spin()
    #rospy.on_shutdown(robot.shutdown())