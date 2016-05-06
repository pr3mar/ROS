#!/usr/bin/env python

import roslib, sys
import rospy
import actionlib
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray

status = 0
nextID = 0
point_list = []
reset = False


def sendGoal():
    global point_list
    pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size = 1)
    print point_list[nextID]
    rospy.loginfo("sending goal")
    pub.publish(point_list[nextID])

def callback(data):
    global status, nextID, reset
    if nextID >= len(point_list):
        sys.exit(0)
    if len(data.status_list) == 0:
        sendGoal()
    stat = data.status_list[len(data.status_list) - 1].status
    print nextID, stat
    if stat == 8: 
        return
    if stat != 1:
        reset = True
    if stat == 1 and reset:
       nextID += 1
       nextID = nextID % len(point_list)
       reset = False
    if status == stat:
        sendGoal()
        return
    if stat == 3 or stat == 4:
        sendGoal()
        status = stat
            

def follower():
    global point_list
    rospy.init_node('sending_multiple_goals')
    x_coord = [3.85, 0.359, -0.0442, -0.365, -0.717, 0.811, 1.19, 1.4]
    y_coord = [-0.372, 0.107, -1.53, -2.89, -4.32, -3.48, -1.89, -0.799]
    for i in range(len(x_coord)):
        tmp = PoseStamped()
        tmp.pose.position.x = x_coord[i]
        tmp.pose.position.y = y_coord[i]
        tmp.pose.position.z = 0
        tmp.pose.orientation.w = 1
        tmp.header.frame_id = 'map'
        tmp.header.stamp = rospy.Time.now()
        point_list.append(tmp)
    # print point_list
    rospy.Subscriber('move_base/status', GoalStatusArray, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        follower()
    except rospy.ROSInterruptException:
        pass


