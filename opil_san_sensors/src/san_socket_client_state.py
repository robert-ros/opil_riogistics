#!/usr/bin/env python

import rospy
from std_srvs.srv import SetBool, SetBoolResponse 
import socket
import sys


RESPONSE_OK  = 0x01
RESPONSE_ERROR = 0x02
RESPONSE_SIZE = 2

class SocketSAN:

    def __init__(self):
        
        # Init node
        rospy.init_node('opil_san_sensors_client', anonymous=False)

        # Get node name
        self.node_name = rospy.get_name()

        # Get ros params
        self.get_ros_params()

        # Create service to send a switch flag
        self.opil_san_sensors_flag = rospy.Service('opil_san_sensors/state_'+str(self.id), SetBool, self.callback_opil_san_sensors_state)

        # Connect to server running in san docker container
        self.san_server_connection()
    
    def get_ros_params(self):

        self.address = rospy.get_param(self.node_name + '/address','localhost')
        self.port = rospy.get_param(self.node_name + '/port',10000)
        self.id = rospy.get_param(self.node_name + '/id', 0)


    def san_server_connection(self):

        # Create a TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Connect the socket to the port where the server is listening
        server_address = (self.address, self.port)
        print('Connecting to {} port {}'.format(*server_address))
        self.sock.connect(server_address)


    def san_server_write(self, message):
        
        print("Sending: [" + message + "]")
        self.sock.sendall(message.encode('utf_8'))

    def san_server_read(self):
        
        status = RESPONSE_OK
        amount_received = 0
        
        last_time = rospy.Time.now().secs
        timeout = 5.0

        while amount_received < RESPONSE_SIZE:
            data = self.sock.recv(RESPONSE_SIZE)
            amount_received += len(data)
            print('Received from server: [{!r}]'.format(data))

            if (rospy.Time.now().secs - last_time > timeout):
                status = RESPONSE_ERROR
                break

        return status, data

    def state_opil_san_sensors(self, state):

        # Send state to san server socket
        if state == True:
            self.san_server_write("True")
        else:
            self.san_server_write("False")
        
        # Get response
        status, data = self.san_server_read()


        if status == RESPONSE_OK:
            message = str(state) + " sent to SAN docker successfully"
            success = True
        else:
            message = "Error sent " + str(state) + " to SAN docker"
            success = False

        return message, success


    def callback_opil_san_sensors_state(self, req):
        
        state = req.data
        result = SetBoolResponse()
        result.message, result.success = self.state_opil_san_sensors(state)
        return result


    def run(self):

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():

            rate.sleep()



if __name__ == '__main__':

    san = SocketSAN()

    try:
        san.run()

    except rospy.ROSInterruptException:
        pass