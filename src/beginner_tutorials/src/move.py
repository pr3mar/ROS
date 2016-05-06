#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class GoForward():
    def __init__(self):
        rospy.init_node('GoForward', annonumous=False)
        sels.cmd_vel = rospy.Publisher('cmd
    def move(step):
        twist = Twist()
        twist.linear.x = 0.4
        step = step % 20
        if step % 5 == 0:
            twist.linear = 0
            twist.angular = 2.8
        return twist

if __name__ == '__main__':
    try:
        GoForward()
    except rospy.ROSInterruptException:
        pass


