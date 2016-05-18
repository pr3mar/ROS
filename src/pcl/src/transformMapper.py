#!/usr/bin/env python
import roslib

roslib.load_manifest('pcl')
import rospy
import sys, select, termios, tty, tf
from visualization_msgs.msg import Marker, MarkerArray
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import Point, Vector3, PoseStamped
import numpy


# Node for face detection.
class MarkerTransformer():
    def faces_callback(self, markers):
        n = len(markers.markers)
        pub = MarkerArray()
        for i in xrange(0, n):
            #print "yay"
            tmp = markers.markers[i]
            #print tmp
            tmpPose = PoseStamped()
            tmpPose.header = tmp.header
            tmpPose.pose = tmp.pose
            try:
                self.listener.waitForTransform(tmp.header.frame_id, '/map', tmp.header.stamp, rospy.Duration(4.5))
                ret = self.listener.transformPose('/map', tmpPose)
            except tf.LookupException:
                print "lookup"
                continue
            except tf.ConnectivityException:
                print "conn"
                continue
            except tf.ExtrapolationException:
                print "extrapolation"
                continue
            tmp.pose = ret.pose
            tmp.header = ret.header
            pub.markers.append(tmp)
        self.markers_pub.publish(pub)


    def __init__(self):
        #self.joined_sub = message_filters.TimeSynchronizer([self.faces_sub, self.camera_sub], 30)
        #self.joined_sub.registerCallback(self.faces_callback)
        self.subs_marker = rospy.Subscriber("/facedetector/markers", MarkerArray, self.faces_callback)
        self.markers_pub = rospy.Publisher('/pcl/transformedMarkers', MarkerArray, queue_size=10)
        self.listener = tf.TransformListener()


# Main function.
if __name__ == '__main__':

    rospy.init_node('facemapper_transformer')
    try:
        fd = MarkerTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
