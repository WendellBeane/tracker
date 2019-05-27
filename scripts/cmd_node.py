#!/usr/bin/python
from tracker.robot import Robot
import rospy
import sys

if __name__ == "__main__":
    rospy.init_node("cmd")
    name = sys.argv[1]
    robot = Robot(name)
    robot.spin()
    rospy.on_shutdown(robot.shutdown())