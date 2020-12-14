#!/usr/bin/env python

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
 
topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray, queue_size=10)

rospy.init_node('register')

markerArray = MarkerArray()


marker = Marker()
marker.id = 1
marker.header.frame_id = "/base_link"
marker.type = marker.CUBE
marker.action = marker.ADD
marker.scale.x = 0.5
marker.scale.y = 0.5
marker.scale.z = 0.5
marker.color.a = 1.0
marker.color.r = 0.0
marker.color.g = 0.0
marker.color.b = 1.0
marker.pose.orientation.x = 0.0
marker.pose.orientation.y = 0.0
marker.pose.orientation.z = 0.0
marker.pose.orientation.w = 1.0
marker.pose.position.x = 0.0
marker.pose.position.y = 0.0 
marker.pose.position.z = 0.05 


markerArray.markers.append(marker)


while not rospy.is_shutdown():


    # Publish the MarkerArray
    publisher.publish(markerArray)

    rospy.sleep(1.0)