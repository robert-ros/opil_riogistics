#!/usr/bin/env python

import SANDriver
import socket
import sys
import time
import threading
import random

MESSAGE_SIZE = 5


# The class has to have the same name that the file 
class cell_2_script(SANDriver.SANDriver):

    def setup(self):

        self.flag = False
        self.setMeta('sensorManufacturer', self.fromConfig("sensorManufacturer"))
        self.setMeta('measurementType', self.fromConfig("measurementType"))
        self.setMeta('sensorType', self.fromConfig("sensorType"))

        self.init_background_san_server(ip_address = 'localhost', port = 10007) 
        self.trigger = False


    def get_reading(self):
          
        return self.trigger


    def init_background_san_server(self, ip_address, port):


        # Create a TCP/IP socket
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Bind the socket to the port
        server_address = (ip_address, port)
        print('Starting up on {} port {}'.format(*server_address))
        self.server_socket.bind(server_address)

        # Listen for incoming connections
        self.server_socket.listen(1)

        # Create san server thread 
        thread_san_server = threading.Thread(target=self.run_background_san_server)
        thread_san_server.daemon = 1
        thread_san_server.start()



    def run_background_san_server(self):

        while True:
            # Wait for a connection
            print('Waiting for a connection..')
            connection, client_address = self.server_socket.accept()
            try:
                print('Connection from', client_address)

                # Receive the data in small chunks and retransmit it
                while True:
                    data = connection.recv(16)
                  
                    print('Received: {!r}'.format(data))
                    if data:
                        response = b'OK'
                        connection.sendall(response)

                        if data == b'True':
                            self.trigger = True
                            print("STATE ON")
                        if data == b'False':
                            self.trigger = False
                            print("STATE OFF")

                    else:
                        print('no data from', client_address)
                        break

            finally:
                # Clean up the connection
                connection.close()






