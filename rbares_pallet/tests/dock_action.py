#!/usr/bin/env python

import rospy
import actionlib
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D, Twist
from robotnik_navigation_msgs.msg import DockAction, DockGoal


if __name__ == '__main__':

    rospy.init_node('robotnik_docker_client')
    client = actionlib.SimpleActionClient('/robot/diff_docker', DockAction)
    client.wait_for_server()

    dock_frame = "robot_pds_pallet_noisy_"
    robot_dock_frame = "robot_dock"
    
    dock_offset = Pose2D()
    dock_offset.x = 0.85
#    dock_offset.y = 0
#    dock_offset.theta = 0

    max_velocity = Twist()
    max_velocity.linear.x = 0.5
    max_velocity.angular.z = 0.5

    goal = DockGoal()
    goal.dock_frame = dock_frame
    goal.robot_dock_frame = robot_dock_frame
    goal.dock_offset = dock_offset
    goal.maximum_velocity = max_velocity


    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))



