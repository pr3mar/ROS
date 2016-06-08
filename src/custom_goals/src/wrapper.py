#!/usr/bin/env python

import nltk, rospy, actionlib, tf
from nltk.metrics import edit_distance
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from std_msgs.msg import String, ColorRGBA
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import *
from visualization_msgs.msg import Marker, MarkerArray
from sound_play.msg import SoundRequest

#global publishers:
markers_pub = None, cancel_pub = None, street_pub = None
slow_pub = None, stop_pub = None, honk_pub = None, oneway_pub = None, voice_pub = None

# all the items that need to be recognized
items = ['peter', 'harry', 'tina', 'scarlett', 'forest', 'kim', 'filip', 'mathew', 'ellen', 'oneway', 'left', 'stop', 'honk', 'limit']

# an array containing all the recognitions (signs/faces)
recognitions = []

# all the detections of faces/signs
# key: coordinates of the marker
# value: [detectionCount, detected, marker, stamps, item]
detections_face = {}
detections_sign = {} # all the detections of signs

def min_distance_all(tokens, find):
    name = None
    original = None
    min_dist = 100
    min_index = None
    for index, i in enumerate(tokens):
        for j in find:
            distance = edit_distance(i, j, transpositions=False)
            if distance < min_dist:
                min_dist = distance
                min_index = index
                name = j
                original = i

    if min_dist > 4:
        return None, 0
    #print "Name we are looking for: %s. It is similar to: %s"%(name, original)
    return  name, min_index

def min_distance_one(original, find):
    #check the colour and name
    if original == None:
        return None
    new = None
    min_dist = 100
    for j in find:
        distance = edit_distance(original, j, transpositions=False)
        if distance < min_dist:
            min_dist = distance
            new = j
    #print "Name we are looking for: %s. It is similar to: %s"%(new, original)
    return  new

def speak_robot(str_speech):
    #this doesnt work the first time - nobody knows why???
    #print str_speech
    tmp = SoundRequest()
    tmp.sound = -3
    tmp.command = 1
    tmp.arg = str_speech
    tmp.arg2 = ''
    global voice_pub
    voice_pub.publish(tmp)

def voice_action(data):
    global search_name, colour_street, street_pub, sent_street, status, cancel_pub, det
    #now do sth with that data!
    names = ['peter', 'harry', 'tina', 'scarlett', 'forest', 'kim', 'filip', 'mathew', 'ellen']
    objects = ['building']
    streets = ['street']
    colours = ['blue', 'yellow', 'green', 'red']
    sentence = data.data

    tokens = nltk.word_tokenize(sentence)
    
    #first word is name
    colour_building = None
    colour_street = None
    name = min_distance_one(tokens[0], names)
    search_name = name

    building = min_distance_all(tokens, objects)
    if building[0] != None:
        colour_building = min_distance_one(tokens[building[1]-1], colours)

    street = min_distance_all(tokens, streets)
    if street[0] != None:
        colour_street = min_distance_one(tokens[street[1]-1], colours)

    print "Mission Impossible: %s %s %s %s %s"%(name,  colour_building, building[0], colour_street, street[0])
    
    just_sent = 0

    if status == 3:
        print "status > 2: This is command from person."
        goto_street_pub.publish(colour_street)
    
    else:
        for key in det:
            if det[key]['name'] == None:
                continue

            name = det[key]['name']
            marker = det[key]['marker']
            #lets check if we already have the face we are looking for (status = 0 means we are looking for a face)
            if search_name != None:
                #print "We have a search name and status is 0."
                if name == search_name:
                    sent_street = 1     #we don't have to send street color, if we know where the face is
                    status = 1
                    just_sent = 1
                    send_pose = marker.pose
                    print "Name we are looking for is the same as the name in our dict. Sending directions. name we are looking for: ", search_name, "Name in our dict: ", name 
                    goal = GoalID()
                    cancel_pub.publish(goal)
                    #alib.cancel_goal()
                    print "canceling in loop, waiting"
                    time = rospy.get_time() + rospy.Duration(1.0).to_sec()
                    while(time > rospy.get_time()):
                        #print "time = ", time, "time now = ", rospy.get_time() 
                        pass
                        
                    goto_pub.publish(send_pose)     #sending coordinates of the person
        if just_sent == 0:
            sent_street = 0
            status = 0

# this makes sure there is a face
def face_detection(detections):
    global detections_face
    redetected = False
    position_threshold = 0.5
    height_threshold = 0.6 # above 0.6 meters it's a false detection
    for marker in detections:
        xp = round(marker.pose.position.x, 2)
        yp = round(marker.pose.position.y, 2)
        zp = round(marker.pose.position.z, 2)
        if zp > height_threshold: # false positive
            continue
        closest_point = None
        for key in detections_face:
            (x, y, z) = key.split(';')
            scatter_x = abs(x - xp)
            scatter_y = abs(y - yp)
            scatter_z = abs(z - zp)
            if scatter_z <= position_threshold \
                and scatter_y <= position_threshold \
                and scatter_z <= position_threshold:
                if not detections_face[key]:
                    closest_point = detections_face[key]
                else:
                    redetected = True
        if closest_point == None: # a new point, add it
            key = str(xp + ";" + yp + ";" + zp)
            detections_face[key] = {"detectionCount": 1, "marker":marker, "detected":False}
        elif:
            closest_point["detectionCount"] += 1
            # TODO: make it work.

# this tells you who is on that particular face
# if the name already exists in the 
def recognize_face(detections):
    pass

# this makes sure there is a sign
def sign_detection(detections):
    pass

# this tells you which sign it is
def recognize_sign(detections):
    pass

# make sure the markers are published all the time
def publish_markers(msg):
    pass

# signals that the goal was reached
def goal_reached(msg):
    pass

if __name__ == '__main__':
    alib = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.init_node('assembled', anonymous=True)
    markers_pub = rospy.Publisher('/viz/markers', MarkerArray, queue_size=100)
    cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=100)
    street_pub = rospy.Publisher('/search/street', String, queue_size=100)
    goto_pub = rospy.Publisher('/goto', Pose, queue_size=100)
    slow_pub = rospy.Publisher('/sign/slow', String, queue_size=100)
    stop_pub = rospy.Publisher('/sign/stop', String, queue_size=100)
    honk_pub = rospy.Publisher('/sign/honk', String, queue_size=100)
    oneway_pub = rospy.Publisher('/sign/oneway', Pose, queue_size=100)
    voice_pub = rospy.Publisher('/robotsound', SoundRequest, queue_size=100)
    rospy.Subscriber('/transformedMarkers/faces', MarkerArray, face_detection)
    rospy.Subscriber('/transformedMarkers/signs', MarkerArray, sign_detection)
    rospy.Subscriber('/recognizer/signs', String, recognize_sign)
    rospy.Subscriber('/recognizer/face', String, recognize_face)
    rospy.Subscriber('/tf', tf.msg.tfMessage, publish_markers)
    rospy.Subscriber('/command', String, voice_action)
    rospy.Subscriber('/goal_reached', String, goal_reached)
    rospy.spin()
