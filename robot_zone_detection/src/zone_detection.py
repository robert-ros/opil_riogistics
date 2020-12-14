#!/usr/bin/env python
import rospy
import math
import os
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, TriggerResponse
import tf
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
import ConfigParser
from tf import TransformListener


class PalletZone:

    def __init__(self):

        # Init node
        rospy.init_node('pallet_zone_detection_node', anonymous=False,  log_level=rospy.WARN)

        self.get_ros_params()

        self.is_pallet_zone_detection_working = False
        self.is_stop_inside = False
        self.last_time = rospy.get_time()
        self.markerArray = MarkerArray()
        self.once_flag = True

        self.listener = tf.TransformListener()
        self.robot_odom = Odometry()

        scrip_path = os.path.dirname(os.path.abspath(__file__)) + "#"
        annotations_path = scrip_path.replace("src#", "config/annotations.ini")
        
        if os.path.exists(annotations_path) == True:
            self.annotations = ConfigParser.ConfigParser()
            self.annotations.read(annotations_path)
        else:
            rospy.logerr("Path " + annotations_path + " does not exist")
            raise SystemExit("Exiting node")

        #self.annotations = ConfigParser.ConfigParser()
        #self.annotations.read('/home/robert/catkin_ws/src/pallet_zone_detection/src/annotations.ini')

        rospy.Subscriber("robotnik_base_control/odom", Odometry, self.callback_odom)
        self.pallet_zone_service = rospy.Service('pallet_zone/state', Trigger , self.callback_pallet_zone_service)
        self.marker_publisher = rospy.Publisher('annotations_marker_array', MarkerArray, queue_size=10)


        if self.use_markers == True:
            self.create_markers()


    def get_ros_params(self):

        self.node_name = rospy.get_name()
        self.global_frame = rospy.get_param(self.node_name + '/global_frame', "map")
        self.base_frame = rospy.get_param(self.node_name + '/base_frame', "base_link")
        self.threshold_stop = rospy.get_param(self.node_name + '/threshold_stop', 0.05 )
        self.threshold_zone = rospy.get_param(self.node_name + '/threshold_zone', 1.0 )
        self.stop_timeout = rospy.get_param(self.node_name + '/stop_timeout', 3.0 )
        self.use_markers = rospy.get_param(self.node_name + '/use_markers', True)

    def waitForMap(self):

        rospy.loginfo("Waiting for transformation beetween " + self.base_frame + " and " + self.global_frame)
        self.listener.waitForTransform(self.base_frame, self.global_frame, rospy.Time(0), rospy.Duration(60*60*24))
        rospy.loginfo("Map received!")

    def create_markers(self):

        count = 0

        for each_section in self.annotations.sections():

            point_x = self.annotations.getfloat(each_section, 'point_x')
            point_y = self.annotations.getfloat(each_section, 'point_y')
            theta = self.annotations.getfloat(each_section, 'theta')
            distance = self.annotations.getfloat(each_section, 'distance')

            x, y = self.get_real_point(point_x, point_y, theta, distance)

            limit = self.threshold_zone 

            x_array = (x, x+limit, x-limit, x+limit, x-limit) 
            y_array = (y, y+limit, y-limit, y-limit, y+limit)

            for index in range(5):
                
                marker = Marker()
                marker.id = count
                marker.header.frame_id = self.global_frame
                marker.type = marker.CUBE
                marker.action = marker.ADD
                marker.scale.x = 0.15
                marker.scale.y = 0.15
                marker.scale.z = 0.15
                marker.color.a = 1.0
                marker.color.b = 1.0
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = x_array[index]
                marker.pose.position.y = y_array[index]
                marker.pose.position.z = 0.05 

                if index == 0:
                    marker.color.r = 1.0
                    marker.color.g = 1.0  
                    marker.color.b = 0.0

                self.markerArray.markers.append(marker)
                count+=1            

                        

    def callback_pallet_zone_service (self, req):

        res = TriggerResponse()

        if self.is_pallet_zone_detection_working == True:

            if self.pallet_zone_errors == False:
                res.message = str(self.robot_dock_yaw) + ":" + str(self.pallet_action)
                res.success = self.pallet_zone_detected
            else:
                res.message="Error processing zone" 
                res.success = False   
        else:
            res.message="Pallet zone detection is not working yet, is the map running?"
            res.success = False

        return res

    def callback_odom(self, msg):

        self.robot_odom = msg
   

    def is_robot_stop(self):

        is_stop = False
        #print(math.fabs(self.robot_odom.twist.twist.linear.x))
        #print(math.fabs(self.robot_odom.twist.twist.angular.z))

        if math.fabs(self.robot_odom.twist.twist.linear.x) <  self.threshold_stop and \
            math.fabs(self.robot_odom.twist.twist.angular.z) < self.threshold_stop:
            is_stop = True

        return is_stop

    def get_real_point(self, point_x, point_y, theta, distance):

        if theta == 0:
            x = point_x - distance
            y = point_y
        elif theta == 90:
            x = point_x
            y = point_y - distance
        elif theta == 180:
            x = point_x + distance
            y = point_y
        elif theta == 270:
            x = point_x
            y = point_y + distance
        else:
            rospy.logerr("Node will not work properly, theta value in the annotations file can only be 0, 90, 180, 270")
            x = 0
            y = 0

        return x, y


    def is_robot_inside_zone(self, robot_pose):

        is_inside = False
        pallet_action = ""
        pallet_yaw = 0

        for each_section in self.annotations.sections():

            point_x = self.annotations.getfloat(each_section, 'point_x')
            point_y = self.annotations.getfloat(each_section, 'point_y')
            theta = self.annotations.getfloat(each_section, 'theta')
            distance = self.annotations.getfloat(each_section, 'distance')
            
            # Distance and theta modifies the final point
            x, y = self.get_real_point(point_x, point_y, theta, distance)

            limit = self.threshold_zone 

            if (robot_pose.x < x+limit) and (robot_pose.x > x-limit) and \
                (robot_pose.y < y+limit) and (robot_pose.y > y-limit):

                is_inside = True
                pallet_yaw = theta*(math.pi/180)
                pallet_action = self.annotations.get(each_section, 'action')
                rospy.logdebug("ROBOT INSIDE A PALLET ZONE!!")
                rospy.logdebug("ACTION IS " + pallet_action)

        return is_inside, pallet_action, pallet_yaw


    def get_robot_pose(self):

        robot_pose = Pose2D()

        self.listener.waitForTransform(self.global_frame, self.base_frame, rospy.Time(), rospy.Duration(4.0))
        (position,orientation) = self.listener.lookupTransform(self.global_frame, self.base_frame, rospy.Time(0))

        euler = euler_from_quaternion(orientation)

        robot_pose.x = position[0]
        robot_pose.y = position[1]
        robot_pose.theta = euler[2] # yaw

        rospy.logdebug("Robot current position is: " + str(robot_pose))
        theta_deg = robot_pose.theta * 180.0 / math.pi
        #rospy.logwarn(theta_deg)

        return robot_pose


    def is_robot_stop_inside_zone(self, robot_pose):

        is_inside, pallet_action, pallet_yaw = self.is_robot_inside_zone(robot_pose)
        is_stop = self.is_robot_stop() 

        if is_stop == True and is_inside == True:
            self.resetTimeOnce()
            if (rospy.get_time() - self.last_time) > self.stop_timeout:
                self.is_stop_inside = True
                rospy.loginfo("ROBOT STOP INSIDE A PALLET ZONE--------!!")

        else:
            self.last_time = rospy.get_time()
            
            self.is_stop_inside = False
            pallet_action = ""

        return self.is_stop_inside, pallet_action, pallet_yaw


    def get_yaw_for_docking(self, robot_yaw, pallet_yaw):
        
        # The frame is inverted because the docking is done in reverse
        
        if robot_yaw > 0:
            robot_yaw = robot_yaw - math.pi
        else:
            robot_yaw = robot_yaw + math.pi

        yaw_counterclockwise = (pallet_yaw - robot_yaw)
        yaw_clockwise = - (2*math.pi - yaw_counterclockwise )

        # Find the shortest way
        if abs(yaw_counterclockwise) > abs(yaw_clockwise):
            robot_dock_yaw = yaw_clockwise 
        else:
            robot_dock_yaw = yaw_counterclockwise

        return robot_dock_yaw


    def pallet_zone_detection(self):
        
        try:
            robot_pose = Pose2D()

            robot_pose = self.get_robot_pose()

            is_stop_inside, pallet_action, pallet_yaw = self.is_robot_stop_inside_zone(robot_pose)
            
            robot_dock_yaw = self.get_yaw_for_docking(robot_pose.theta, pallet_yaw)

            if is_stop_inside == True:

                self.pallet_action = pallet_action
                self.robot_dock_yaw = round(robot_dock_yaw, 7)
                self.pallet_zone_detected = True
            
            else:
                self.pallet_action = pallet_action
                self.robot_dock_yaw = 0
                self.pallet_zone_detected = False
       

            self.pallet_zone_errors = False
            self.is_pallet_zone_detection_working = True

        except Exception as e:
            # rospy.logerr(e.message)
            self.pallet_zone_errors = True
            pass


    def resetTimeOnce(self):
        
        if self.once_flag == True:
            self.last_time = rospy.get_time()
            self.once_flag = False

    def run(self):
        
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():

            self.pallet_zone_detection()

            if self.use_markers == True:
                self.marker_publisher.publish(self.markerArray)

            rate.sleep()




if __name__ == '__main__':

    pallet_zone = PalletZone()

    try:
        pallet_zone.run()

    except rospy.ROSInterruptException:
        pass
