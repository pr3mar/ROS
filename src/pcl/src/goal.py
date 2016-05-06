#!/usr/bin/env python
# license removed for brevity

import roslib; 
import rospy
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi

def move():
    pub = rospy.Publisher('/move_base_simple/goal', MoveBaseAction, queue_size = 10)
    rospy.init_node('move', anonymous=True)
    rospy.loginfo("Moving...")
  
    location= Pose(Point(1.04, 0.25, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))
    self.goal = MoveBaseGoal()
    self.goal.target_pose.pose = location
    self.goal.target_pose.header.frame_id = 'map'
    self.goal.target_pose.headerstamp = rospy.Time.now()
        
    self.move_base.send_goal(self.goal)
        
    time = self.move_base.wait_for_result(rospy.Duration(300))
        
    
    
    pub.publish(msg)
    rate.sleep()

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
