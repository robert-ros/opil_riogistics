#!/usr/bin/env python


import rospy
from robotnik_msgs.msg import inputs_outputs
from std_srvs.srv import Trigger, TriggerRequest 
from std_srvs.srv import SetBool, SetBoolRequest

class EmkaModbus:

	def __init__(self):

		# Init node
		rospy.init_node('emka_modbus_node')
		rospy.loginfo("Starting program...")

		self.get_params()

		self.sensor_states = [False, False, False, False, False, False, False, False, False]

		# Subscribe to cmd_vel robot
		rospy.Subscriber("robotnik_modbus_io/input_output", inputs_outputs, self.modbus_callaback, queue_size=1)

		rospy.wait_for_service('san_socket/trigger_button_g1')
		rospy.wait_for_service('san_socket/trigger_button_r1')
		rospy.wait_for_service('san_socket/trigger_button_g2')
		rospy.wait_for_service('san_socket/trigger_button_r2')

		self.trigger_button_g1 = rospy.ServiceProxy('/san_socket/trigger_button_g1', Trigger)
		self.trigger_button_r1 = rospy.ServiceProxy('/san_socket/trigger_button_r1', Trigger)
		self.trigger_button_g2 = rospy.ServiceProxy('/san_socket/trigger_button_g2', Trigger)
		self.trigger_button_r2 = rospy.ServiceProxy('/san_socket/trigger_button_r2', Trigger)


		rospy.wait_for_service('san_socket/state_cell_1')
		rospy.wait_for_service('san_socket/state_cell_2')
		rospy.wait_for_service('san_socket/state_cell_3')

		self.state_cell_1 = rospy.ServiceProxy('/san_socket/state_cell_1', SetBool)
		self.state_cell_2 = rospy.ServiceProxy('/san_socket/state_cell_2', SetBool)
		self.state_cell_3 = rospy.ServiceProxy('/san_socket/state_cell_3', SetBool)

		rospy.loginfo("Running example!")


	def get_params(self):

		self.BUTTON_G1 = rospy.get_param("plc/BUTTON_G1")
		self.BUTTON_R1 = rospy.get_param("plc/BUTTON_R1")
		self.BUTTON_G2 = rospy.get_param("plc/BUTTON_G2")
		self.BUTTON_R2 = rospy.get_param("plc/BUTTON_R2")
		self.DOOR_UP = rospy.get_param("plc/DOOR_UP")
		self.DOOR_DOWN = rospy.get_param("plc/DOOR_DOWN")
		self.CELL_1 = rospy.get_param("plc/CELL_1")
		self.CELL_2 = rospy.get_param("plc/CELL_2")
		self.CELL_3 = rospy.get_param("plc/CELL_3")


	def modbus_callaback(self, msg):

		print("---------------")

		self.sensor_states = msg.digital_inputs

		print(self.sensor_states)
		print(self.sensor_states[self.BUTTON_G1])
		print(self.sensor_states[self.BUTTON_R1])
		print(self.sensor_states[self.BUTTON_G2])
		print(self.sensor_states[self.BUTTON_G2])

	def run(self):

		req = TriggerRequest()

		if self.sensor_states[self.BUTTON_G1] == True:
			self.trigger_button_g1(req)
		
		if self.sensor_states[self.BUTTON_R1] == True:
			self.trigger_button_r1(req)

		if self.sensor_states[self.BUTTON_G2] == True:
			self.trigger_button_g2(req)
		
		if self.sensor_states[self.BUTTON_R2] == True:
			self.trigger_button_r2(req)


		req_state = SetBoolRequest()

		req_state.data = self.sensor_states[self.CELL_1]
		self.state_cell_1(req_state)

		req_state.data = self.sensor_states[self.CELL_2]
		self.state_cell_2(req_state)

		req_state.data = self.sensor_states[self.CELL_3]
		self.state_cell_3(req_state)	

def main():
	
	emka_modbus = EmkaModbus()
	r = rospy.Rate(2)

	while not rospy.is_shutdown():

		emka_modbus.run()
		r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass