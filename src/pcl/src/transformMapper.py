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
        n = len(markers)
        for i in xrange(0, n):
            tmp = markers[i]
            tmpPose = tmp.pose
            try:
                self.listener.waitForTransform(tmp.header.frame_id, '/map', tmp.header.stamp, rospy.Duration(4.5))
                tmpPose = self.listener.transformPose('/map', tmpPose)
            except tf.LookupException:
                print
                "lookup"
                continue
            except tf.ConnectivityException:
                print
                "conn"
                continue
            except tf.ExtrapolationException:
                print
                "extrapolation"
                continue
            tmp.pose = tmpPose
            markers[i] = tmp
        self.markers_pub.publish(markers)


def __init__(self):
    self.joined_sub = message_filters.TimeSynchronizer([self.faces_sub, self.camera_sub], 30)
    self.joined_sub.registerCallback(self.faces_callback)
    self.markers_pub = rospy.Publisher('/pcl/transformedMarkers', MarkerArray)
    self.listener = tf.TransformListener()


# Main function.
if __name__ == '__main__':

    rospy.init_node('facemapper_transformer')
    try:
        fd = MarkerTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
