#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist

def talker():
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(50) # 10hz
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
    move_cmd = Twist()
    #move_cmd.linear.x = 0.25
    #move_cmd.angular.z = 0
    move_cmd.angular.z = 1
    counter = 0
    while not rospy.is_shutdown():
        pub.publish(move_cmd)        
#        if counter > 82:
        if counter > 328:
            break
        counter += 1
	rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


