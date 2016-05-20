#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sound_play.msg import SoundRequest

def callback(data):
    print "You said: "+data.data

    #now do sth with that data!
    #this doesnt work the first time
    tmp = SoundRequest()
    tmp.sound = -3
    tmp.command = 1
    tmp.arg = 'Marko I am so sorry but I will not be able to come in tomorrow, please do not kill me! This is not a joke. Your dear friend Jacob'
    tmp.arg2 = ''
    
    pub = rospy.Publisher('/robotsound', SoundRequest, queue_size=1)
    pub.publish(tmp)
    
    
def commander():

    rospy.init_node('commander', anonymous=True)
    rospy.Subscriber("/command", String, callback)
    
    
    rospy.spin()

if __name__ == '__main__':
    commander()
