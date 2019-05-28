#!/usr/bin/python
from tracker.control_robot import Control_Robot
import rospy

if __name__ == "__main__":
    rospy.init_node("cmd")
    robot = Control_Robot()
    robot.spin()
    rospy.on_shutdown(robot.shutdown())