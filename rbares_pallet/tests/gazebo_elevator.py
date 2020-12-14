#!/usr/bin/env python

import rospy
from robotnik_msgs.srv import SetElevator, SetElevatorRequest, SetElevatorResponse
from std_msgs.msg import Float64


class Elevator:

    def __init__(self):

        rospy.init_node('elevator_node', anonymous=True)
        self.elevator_controller = rospy.Publisher('/robot/elevator_controller/command', Float64  , queue_size=1)
        self.set_elevator = rospy.Service('/robot/robotnik_base_control/SetElevator', SetElevator, self.callback_set_elevator)
        

    def callback_set_elevator(self, req):

        if req.action.action == 1:
            rospy.loginfo("Elevator UP")
            self.elevator_controller.publish(1)
        else:
            rospy.loginfo("Elevator DOWN")
            self.elevator_controller.publish(-1)

        res = SetElevatorResponse()
        res.ret = True

        rospy.sleep(3)

        return res

    def run(self):

        rospy.spin()


if __name__ == '__main__':

    elevator = Elevator()

    try:
        elevator.run()

    except rospy.ROSInterruptException:
        pass

