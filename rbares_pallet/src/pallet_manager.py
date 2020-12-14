#!/usr/bin/env python

import rospy
import actionlib

from geometry_msgs.msg import Pose2D, Twist
from robotnik_navigation_msgs.msg import MoveAction, MoveGoal
from robotnik_navigation_msgs.msg import DockAction, DockGoal
from robotnik_msgs.srv import SetElevator, SetElevatorRequest, SetElevatorResponse
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

from Tkinter import *
import ttk
import ScrolledText 

from tf import TransformListener


UP = 1
DOWN = -1

MOVE_ANGLE = 0
MOVE_DISTANCE = 1
MOVE_ANGLE_BACK = 2
MOVE_DISTANCE_BACK = 3
DETECT_PALLET = 4
DOCK = 5
ELEVATOR = 6
MOVE_OFFSET = 7

START = 0
FIND_ZONE = 1
SELECT_MODE = 2
SEMI = 3
AUTO = 4
FINISHED = 5

# Preparar respuesta entrevista
# Terminal excel facturas
# Preparar video OPIL


class LoadPallet:


    def __init__(self):

        rospy.init_node('pallet_manager_node', anonymous=True)

        self.get_rosparam()

        self.init_move()
        self.init_detect()
        self.init_dock()
        self.init_elevator()
        self.init_zone()
        self.init_gui()

    def get_rosparam(self):

        self.node_name = rospy.get_name()
        self.use_sim= rospy.get_param(self.node_name + '/use_sim', True)     
        self.route= rospy.get_param(self.node_name + '/route', 1)   

    def init_detect(self):

        self.listener = TransformListener()

    def init_move(self):

        self.move_client = actionlib.SimpleActionClient('move', MoveAction)
        self.move_client.wait_for_server()        


    def init_dock(self):

        self.dock_client = actionlib.SimpleActionClient('diff_docker', DockAction)
        self.dock_client.wait_for_server()

    def init_elevator(self):

        rospy.wait_for_service('robotnik_base_control/set_elevator')
	if self.use_sim == True:
            self.elevator_client = rospy.ServiceProxy('robotnik_base_control/SetElevator', SetElevator)
        else:
            self.elevator_client = rospy.ServiceProxy('robotnik_base_control/set_elevator', SetElevator)

    def init_zone(self):

        #rospy.wait_for_service('pallet_zone/state')
        self.zone_client = rospy.ServiceProxy('pallet_zone/state', Trigger)


    def init_gui(self):

        self.window = Tk()
        w = 425
        h = 300
        ws = self.window.winfo_screenwidth()
        hs = self.window.winfo_screenheight()

        x = (ws/2) - (w/2)
        y = -100 + ((hs/2) - (h/2))

        self.window.geometry('%dx%d+%d+%d' % (w, h, x, y))
        self.window.title("Docking Assistant")

        # Move distance GUI
        self.moveDistanceButton = Button(self.window, text="Move distance", command=self.moveDistanceButton)
        self.moveDistanceButton.place(x = 20, y = 20, width=120, height=25)

        self.moveDistanceEntry = Entry(self.window)
        self.moveDistanceEntry.place(x = 150, y = 20,  width=120, height = 25)
        self.moveDistanceEntry.insert(END, '')
        self.moveDistanceEntry.insert(END, '-0.05')

        # Move angle GUI
        self.moveAngleButton = Button(self.window, text="Move angle", command=self.moveAngleButton)
        self.moveAngleButton.place(x = 20, y = 60, width=120, height=25)

        self.moveAngleEntry = Entry(self.window)
        self.moveAngleEntry.place(x = 150, y = 60,  width=120, height = 25)
        self.moveAngleEntry.insert(END, '')

        # Detect pallet GUI
        self.detectPalletButton = Button(self.window, text="Detect pallet", command=self.detectPalletButton)
        self.detectPalletButton.place(x = 20, y = 100, width=120, height=25)

        self.detectPalletLabel = Label(self.window, text="Not detected")
        self.detectPalletLabel.place(x = 150, y = 100)

        # Dock GUI
        self.dockButton = Button(self.window, text="Dock", command=self.dockButton)
        self.dockButton.place(x = 20, y = 140, width=120, height=25)

        self.dockEntry = Entry(self.window)
        self.dockEntry.place(x = 150, y = 140,  width=120, height = 25)
        self.dockEntry.insert(END, '0.85')

        # Elevate pallet GUI
        self.elevatePalletButton = Button(self.window, text="Elevate pallet", command=self.elevatePalletButton)
        self.elevatePalletButton.place(x = 20, y = 180, width=120, height=25)

        self.elevatePalletCombo = StringVar()
        self.elevatePalletCombo = ttk.Combobox(self.window, textvariable = self.elevatePalletCombo, width=6, height=10)
        self.elevatePalletCombo['values'] = ('UP', 'DOWN')
        self.elevatePalletCombo.place(x = 150, y = 180,  width=120, height=25)
        self.elevatePalletCombo.current(0)

        # Zone detection GUI
        self.zoneDetectionButton = Button(self.window, text="Zone", command=self.zoneDetectionButton)
        self.zoneDetectionButton.place(x = 285, y = 20, width=120, height=25)

        self.zoneDetectionYawLabel = Label(self.window, text="Yaw: ???")
        self.zoneDetectionYawLabel.place(x = 282, y = 52)
        
        self.zoneDetectionActionLabel = Label(self.window, text="Action: ???")
        self.zoneDetectionActionLabel.place(x = 282, y = 72)

        
        # Semi-autonomous GUI
        self.semiAutonomousButton = Button(self.window, text="Run", command=self.semiAutonomousButton)
        self.semiAutonomousButton.place(x = 285, y = 130, width=120, height=25)

        self.semiAutonomousLabel = Label(self.window, text="Semi-autonomous")
        self.semiAutonomousLabel.place(x = 285, y = 107)

        # Autonomous GUI
        self.autonomousButton = Button(self.window, text="Run", command=self.autonomousButton)
        self.autonomousButton.place(x = 285, y = 180, width=120, height=25)

        self.autonomousLabel = Label(self.window, text="Autonomous")
        self.autonomousLabel.place(x = 285, y = 157)


        self.semiAutonomousFlag = False
        self.autonomousFlag = False
        self.run_state = START
        self.load_state = MOVE_ANGLE
        self.unload_state = MOVE_ANGLE
        self.zone_yaw = 0
        self.zone_action = ""

        # Reset states GUI
        self.resetStateButton = Button(self.window, text="reset", command=self.resetStateButton)
        self.resetStateButton.place(x = 365, y = 80, width=50, height=20)

        # Scroll Text GUI
        self.text_area = ScrolledText.ScrolledText(self.window, font = ("Ubuntu Regular", 11)) 
        self.text_area.place(x = 20, y = 215, width = 390, height = 75)
        self.text_area.focus() 

        self.pallet_distance = 0

    def moveDistanceButton(self):

        distance = float(self.moveDistanceEntry.get())
        self.print_info("Received from GUI: move distance " + str(distance) + " m")
        self.move_foward(distance)

    def moveAngleButton(self):

        angle = float(self.moveAngleEntry.get())
        self.print_info("Received from GUI: move angle " + str(angle) + " rad")
        self.move_turn(angle)

    def detectPalletButton(self):

        found, distance = self.detect_pallet()

        if found == True:
            self.detectPalletLabel.config(text='"Detected!')
        else:
            self.detectPalletLabel.config(text='Not detected')

    def dockButton(self):

        x_offset =  float(self.dockEntry.get())
        self.print_info("Received from GUI: dock action with of" + str(x_offset) +  " m X offset")
        self.dock_run(x_offset)

    def elevatePalletButton(self):

        elevate = self.elevatePalletCombo.get()
        self.print_info("Received from GUI: elevate " + elevate)

        if elevate == "UP":
            self.set_elevator(1)
        if elevate == "DOWN":
            self.set_elevator(-1)

    def zoneDetectionButton(self):
        
        zone_success, zone_yaw, zone_action = self.detect_zone()

        if zone_success == True:
            self.zoneDetectionYawLabel.config(text='Yaw: ' + str(zone_yaw))
            self.zoneDetectionActionLabel.config(text='Action: ' + zone_action )
            self.moveAngleEntry.delete(0,END)
            self.moveAngleEntry.insert(0, str(zone_yaw))
        else:
            self.zoneDetectionYawLabel.config(text='Yaw: ???')
            self.zoneDetectionActionLabel.config(text='Action: ???')


    # Semi Autonomous button functions

    def semiAutonomousButton(self):

        self.semiAutonomousFlag = True

    def readSemiAutonomousButton(self):

        aux = self.semiAutonomousFlag
        self.semiAutonomousFlag = False

        return aux

    def waitSemiAutonomousModeButton(self):

        self.print_info("Waiting run flag...")
        while self.semiAutonomousFlag == False:
            self.update_gui() 
            rospy.sleep(0.1)
        
        self.semiAutonomousFlag = False

    # ----------------------------------    

    # SemiAutonomous button functions

    def autonomousButton(self):

        self.autonomousFlag = True

    def readAutonomousButton(self):

        aux = self.autonomousFlag
        self.autonomousFlag = False

        return aux

    # -----------------------------------

    def resetStateButton(self):

        self.load_state = MOVE_ANGLE
        self.unload_state = MOVE_ANGLE
        self.run_state = START
        self.semiAutonomousFlag = False
        self.autonomousFlag = False
        self.print_info("All state machines have been reset ")

    
    def print_info(self, text):

        self.text_area.insert("end", text+'\n')
        self.text_area.see("end")
        self.update_gui()
        rospy.loginfo(text)


    def print_warn(self, text):

        self.text_area.insert("end", text+'\n')
        self.text_area.see("end")
        self.update_gui()
        rospy.logwarn(text)

    def move_turn(self, theta):

        goal_position = Pose2D()
        goal_position.x = 0.0
        goal_position.theta = theta

        max_velocity = Twist()
        max_velocity.linear.x = 0.5
        max_velocity.angular.z = 0.5

        goal = MoveGoal()
        goal.goal = goal_position
        goal.maximum_velocity = max_velocity

        self.move_client.send_goal(goal)
        self.move_client.wait_for_result()


    def move_foward(self, distance):

        goal_position = Pose2D()
        goal_position.x = distance
        goal_position.theta = 0.0

        max_velocity = Twist()
        max_velocity.linear.x = 0.5
        max_velocity.angular.z = 0.5

        goal = MoveGoal()
        goal.goal = goal_position
        goal.maximum_velocity = max_velocity

        self.move_client.send_goal(goal)
        self.move_client.wait_for_result()


    def detect_pallet(self):

        try:
            now = rospy.Time(0)
            self.listener.waitForTransform("/robot_pds_pallet_noisy_", "/robot_base_link", now, rospy.Duration(5.0))
            (trans,rot) = self.listener.lookupTransform('/robot_pds_pallet_noisy_', '/robot_base_link', rospy.Time(0))
            distance = -trans[0]
            found = True
        except Exception as e:
            distance = 0
            found = False
            print(e)

        self.listener.clear()

        return found, distance

    def dock_run(self, offset_x = 0.85):

        dock_frame = "robot_pds_pallet_noisy_"
        robot_dock_frame = "robot_dock"
        
        dock_offset = Pose2D()
        dock_offset.x = offset_x
        #dock_offset.y = 0
        #dock_offset.theta = 0

        max_velocity = Twist()
        max_velocity.linear.x = 0.5
        max_velocity.angular.z = 0.5

        goal = DockGoal()
        goal.dock_frame = dock_frame
        goal.robot_dock_frame = robot_dock_frame
        goal.dock_offset = dock_offset
        goal.maximum_velocity = max_velocity


        self.dock_client.send_goal(goal)
        self.dock_client.wait_for_result()


    def set_elevator(self, state):
	
        rospy.sleep(3)
        try:
           req_elevator = SetElevatorRequest()
           req_elevator.action.action = state
           self.elevator_client(req_elevator)
        except Exception as e:
           self.print_warn("I can not set elevator because I am moving");
           print(e)

    def detect_zone(self):

        req_zone = TriggerRequest()
        res_zone = TriggerResponse()

        zone_yaw = 0.0
        zone_action = ""
        zone_success = False

        try:
            res_zone = self.zone_client(req_zone)

            zone_success = res_zone.success

            if zone_success == True:
                zone_yaw = float(res_zone.message.split(':')[0])
                zone_action = res_zone.message.split(':')[1] 
        except:
            zone_success = False

        return zone_success, zone_yaw, zone_action
    

    def semiAutonomousMode(self, zone_yaw, zone_action):

        finished = False

        if zone_action == "load":
            finished = self.load_pallet(zone_yaw, mode="semi-autonomous")
        if zone_action == "unload":
            finished = self.unload_pallet(zone_yaw, mode="semi-autonomous")

        return finished


    def autonomousMode(self, zone_yaw, zone_action):

        finished = False

        if zone_action == "load":
            finished = self.load_pallet(zone_yaw, mode="autonomous")
        if zone_action == "unload":
            finished = self.unload_pallet(zone_yaw, mode="autonomous")

        return finished


    def load_pallet(self, zone_yaw, mode):

        finished = False
        # pallet_a: -2.8
        # pallet_c: -2.5
	if self.route == 1:
            move_distance = -2.8 
        else:
            move_distance = -2.5

        dock_offset = -0.2  
        move_offset = -0.975

        total_move_backard = move_distance

        if mode == "semi-autonomous":
            self.waitSemiAutonomousModeButton()

        
        if self.load_state == MOVE_ANGLE:

            self.print_info(mode + " mode: move angle " + str(zone_yaw) + " rad")
            self.move_turn(zone_yaw)
            self.load_state = MOVE_DISTANCE


        elif self.load_state == MOVE_DISTANCE:

            self.print_info(mode + " mode: move distance " + str(move_distance) + " m")
            self.move_foward(move_distance) # Negative, backward
            self.load_state = DETECT_PALLET
            
        elif self.load_state == DETECT_PALLET:   

            self.print_info(mode + " mode: detecting pallet")
            found, self.pallet_distance = self.detect_pallet() 
            count = 0
            while found != True:
                count+=1
                self.print_info(mode + " mode: pallet not detected, try again " + str(count))
                if mode == "semi-autonomous":
                    self.waitSemiAutonomousModeButton()
                else:
                    rospy.sleep(1.0)
                found, self.pallet_distance = self.detect_pallet() 
            self.print_info(mode + " mode: pallet detected in " + str(self.pallet_distance) + " m !")
            self.load_state = DOCK


        elif self.load_state == DOCK:

            self.print_info(mode + " mode: dock action with of " + str(dock_offset) + " m X offset")
            self.dock_run(dock_offset)
            self.load_state = MOVE_OFFSET

        elif self.load_state == MOVE_OFFSET:
             
            self.print_info(mode + " mode: move offset with " + str(move_offset) + " m !")
            self.move_foward(move_offset);
            self.load_state = ELEVATOR
 
        elif self.load_state == ELEVATOR:

            self.print_info(mode + " mode: elevate UP")
            self.set_elevator(1)
            self.load_state = MOVE_DISTANCE_BACK


        elif self.load_state == MOVE_DISTANCE_BACK:

            self.print_info(mode + " mode: move distance " + str(-move_distance+ self.pallet_distance - move_offset + dock_offset) + " m")
            self.move_foward(-move_distance+ self.pallet_distance - move_offset + dock_offset)
            self.load_state = MOVE_ANGLE_BACK


        elif self.load_state == MOVE_ANGLE_BACK:

            self.print_info(mode + " mode: move angle " + str(-zone_yaw) + " rad")
            self.move_turn(-zone_yaw)
            self.load_state = MOVE_ANGLE
            self.print_info(mode + " mode finished")
            finished = True


        return finished


    def unload_pallet(self, zone_yaw, mode):

        finished = False
        
        #move_distance = -3.2
        #dock_offset = -0.2
        #move_offset = -1.0 

	#total_move_backward = -(move_distance + move_offset - dock_offset)
        #total_move_foward = move_distance + move_offset - dock_offset


        # pallet_b: 4.2
        # pallet_d: 2.3
        if self.route == 1:
            total_move_backward = -4.2 
            total_move_foward = 4.2
        else:
            total_move_backward = -2.3 
            total_move_foward = 2.3

        if mode == "semi-autonomous":
            self.waitSemiAutonomousModeButton()

        if self.unload_state == MOVE_ANGLE:
             
            self.print_info(mode + " mode: move angle " + str(zone_yaw) + " rad")
            self.move_turn(zone_yaw)
            self.unload_state = MOVE_DISTANCE


        elif self.unload_state == MOVE_DISTANCE:

            self.print_info(mode + " mode: move distance " + str(total_move_backward) + " m")
            self.move_foward(total_move_backward) # Negative, backward
            self.unload_state = ELEVATOR
            

        elif self.unload_state == ELEVATOR:
            
            self.print_info(mode + " mode: elevate DOWN")
            self.set_elevator(-1)
            self.unload_state = MOVE_DISTANCE_BACK


        elif self.unload_state == MOVE_DISTANCE_BACK:

            self.print_info(mode + " mode: move distance " + str(total_move_foward) + " m")
            self.move_foward(total_move_foward)
            self.unload_state = MOVE_ANGLE_BACK

    
        elif self.unload_state == MOVE_ANGLE_BACK:

            self.print_info(mode + " mode: move angle " + str(-zone_yaw) + " rad")
            self.move_turn(-zone_yaw)
            self.unload_state = MOVE_ANGLE
            finished = True


        return finished
        

    def update_gui(self):

        self.window.update()   


    def run(self):

        self.update_gui()  

        if self.run_state == START:

            self.print_info("Finding new zone...")
            self.run_state = FIND_ZONE

        if self.run_state == FIND_ZONE:

            zone_success, zone_yaw, zone_action = self.detect_zone()
            
            if zone_success == True:
                self.print_info(zone_action + " zone detected!")
                self.print_info("Select a mode: auto or semi-auto")
                self.run_state = SELECT_MODE

            if zone_success == False:
                if self.readSemiAutonomousButton() == True or self.readAutonomousButton() == True:
                    self.print_warn("Robot is not inside a zone")


        elif self.run_state == SELECT_MODE:
            
            zone_success, self.zone_yaw, self.zone_action = self.detect_zone()

            if zone_success == False:
                self.run_state = START
                self.print_info("Robot has been moved")

            if self.readSemiAutonomousButton() == True:
                self.print_info("Starting semi-autonomous mode")
                self.run_state = SEMI

            if self.readAutonomousButton() == True:
                self.print_info("Starting autonomous mode")
                self.run_state = AUTO


        elif self.run_state == SEMI:

            if self.semiAutonomousMode(self.zone_yaw, self.zone_action) == True:      
                self.run_state = FINISHED


        elif self.run_state == AUTO:

            if self.autonomousMode(self.zone_yaw, self.zone_action) == True:
                self.run_state = FINISHED


        elif self.run_state == FINISHED:
            self.run_state = START
            



def main():

    load_pallet = LoadPallet()
    r = rospy.Rate(20)

    while not rospy.is_shutdown():
        
        load_pallet.run()
        r.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
