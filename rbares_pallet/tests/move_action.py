#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Pose2D, Twist
from robotnik_navigation_msgs.msg import MoveAction, MoveGoal

# Notas
# La posicion que se envia es un incremento respecto a su posicion actual
# El goal solo puede tener componente en X o componente en Theta, pero nunca ambos a la vez

if __name__ == '__main__':

    rospy.init_node('robotnik_move_client')
    client = actionlib.SimpleActionClient('/robot/move', MoveAction)
    client.wait_for_server()
    
    goal_position = Pose2D()
    goal_position.x = 0.0
    goal_position.theta = -3.14

    max_velocity = Twist()
    max_velocity.linear.x = 0.5
    max_velocity.angular.z = 0.5

    goal = MoveGoal()
    goal.goal = goal_position
    goal.maximum_velocity = max_velocity


    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))