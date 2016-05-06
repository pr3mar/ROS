#!/usr/bin/env python
import rospy
from visualization_msgs.msg import *
from geometry_msgs.msg import *

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.markers)
    
def listener_face():

   
    rospy.init_node('listener_face', anonymous=True)

    rospy.Subscriber("/facemapper/markers", MarkerArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener_face()
