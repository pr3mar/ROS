#!/usr/bin/env python

import roslib, sys
import rospy
import actionlib
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray
from visualization_msgs.msg import Marker, MarkerArray


class SendGoal():
    def detection_thresh(self, points):
        for newPoint in points.markers:
            # newPoint = points[n]
            position = newPoint.pose.position
            xp = round(position.x, 2)
            yp = round(position.y, 2)
            zp = round(position.z, 2)
            mind = 0.5
            min_point = None
            if zp > 0.40:
                print xp, yp, zp
                continue
            for key in self.det:
                (x, y, z) = key.split(';')
                dist_x = abs(xp - float(x))
                dist_y = abs(yp - float(y))
                dist_z = abs(zp - float(z))
                if dist_x <= mind and dist_y <= mind and dist_z <= mind:
                    if not self.det[key]['detected']:
                        min_point = self.det[key]
                    else:
                        min_point = "not"

            if min_point == None:
                self.det[str(xp) + ";" + str(yp) + ";" + str(zp)] = {'count': 1, 'detected': False,
                                                                     'point': str(xp) + ";" + str(yp) + ";" + str(zp)}
            elif min_point != "not":
                min_point['count'] += 1
                if min_point['count'] > 25:
                    min_point['detected'] = True
                    self.detected += 1
                    print "face " + str(self.detected) + " detected!!!!\n" + str(min_point)
                    print self.det
            else:
                pass

    def sendGoal(self):
        pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)
        # print self.point_list[self.nextID]
        rospy.loginfo("sending goal" + str(self.nextID))  # str(self.point_list[self.nextID]))
        pub.publish(self.point_list[self.nextID])

    def callback(self, data):
        if self.nextID >= len(self.point_list):
            sys.exit(0)
        if len(data.status_list) == 0:
            self.sendGoal()
            return
        # print len(data.status_list)
        stat = data.status_list[len(data.status_list) - 1].status
        # print self.nextID, stat
        if stat == 8:
            return
        if stat != 1:
            self.reset = True
        if stat == 1 and self.reset:
            self.nextID += 1
            self.nextID = self.nextID % len(self.point_list)
            self.reset = False
        if self.status == stat:
            self.sendGoal()
            return
        if stat == 3 or stat == 4:
            self.sendGoal()
            self.status = stat

    def __init__(self):
        rospy.init_node('sending_multiple_goals')
        # x_coord = [3.85, 0.359, -0.0442, -0.365, -0.717, 0.811, 1.19, 1.4]
        # y_coord = [-0.372, 0.107, -1.53, -2.89, -4.32, -3.48, -1.89, -0.799]
        x_coord = [3.702, 3.748, 3.748, 3.748, 2.332, 0.887, 0.295, -0.029, -0.164, -0.554, -0.752, 0.637, 0.856, 1.029,
                   2.498]
        y_coord = [0.137, -0.362, -0.362, -0.362, -0.044, -0.171, -0.004, -1.000, -2.184, -3.065, -4.190, -3.298,
                   -2.084, -1.081, -0.037]
        z_coord = [0.649, -0.065, -0.726, 0.686, 0.649, 0.649, 0.984, 0.979, 0.996, 0.984, -0.820, -0.731, -0.078,
                   -0.078, -0.734]
        w_coord = [0.760, 0.997, 0.686, 0.726, 0.76, 0.760, 0.176, 0.199, 0.078, 0.176, 0.571, 0.681, 0.996, 0.996,
                   0.678]
        self.point_list = []
        for i in range(len(x_coord)):
            tmp = PoseStamped()
            tmp.pose.position.x = x_coord[i]
            tmp.pose.position.y = y_coord[i]
            tmp.pose.position.z = 0
            tmp.pose.orientation.z = z_coord[i]
            tmp.pose.orientation.w = w_coord[i]
            tmp.header.frame_id = 'map'
            tmp.header.stamp = rospy.Time.now()
            self.point_list.append(tmp)
            self.status = 0
        self.nextID = 0
        self.reset = False
        self.det = {}
        self.detected = 0
        rospy.Subscriber('facedetector/markers', MarkerArray, self.detection_thresh)
        rospy.Subscriber('move_base/status', GoalStatusArray, self.callback)
        rospy.spin()


if __name__ == '__main__':
    try:
        fl = SendGoal()
    except rospy.ROSInterruptException:
        pass
