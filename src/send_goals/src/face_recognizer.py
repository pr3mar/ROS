#!/usr/bin/env python

import nltk
from nltk.metrics import edit_distance
import rospy
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import *
from visualization_msgs.msg import Marker, MarkerArray
from sound_play.msg import SoundRequest
import tf

status = 0

voice_pub = None
markers_pub = None
cancel_pub = None
goto_pub = None
slow_pub = None 
stop_pub = None
honk_pub = None
oneway_pub = None
det = {}
search_name = None

#faces
detected = 0
detect_true = 0
faces_count = {}
sound_sent = 0
color = None
det_entry = None

#signs
det_signs = {}
sign_detected = 0
sign_detected_again = 0
detect_sign_true = 0
det_entry_sign = None
signs_count = {}
color_map = {'honk': ColorRGBA(85, 96, 240, 1), 'left': ColorRGBA(16, 31, 239, 1), 'limit': ColorRGBA(249, 98, 98, 1), 'oneway': ColorRGBA(245, 1, 1, 1), 'stop': ColorRGBA(152, 6, 6, 1), 'peter': ColorRGBA(128, 255, 0, 1), 'tina': ColorRGBA(51, 51, 255, 1), 'harry': ColorRGBA(242, 208, 15, 1), 'forest': ColorRGBA(255, 128, 0, 1), 'filip': ColorRGBA(102, 51, 0, 1), 'kim': ColorRGBA(19, 237, 226, 1), 'mathew': ColorRGBA(203, 17, 240, 1), 'scarlett': ColorRGBA(111, 255, 0, 1), 'ellen': ColorRGBA(10, 107, 107, 1)}
# honk: light blue, left: dark blue, limit: pink, oneway: light red, stop: dark red, peter: light green, tina: dark green, harry: light orange,  forest: orange, filip: brown, kim: tirquoise, mathew: purple, scarlett: light green, ellen: dark green-blue 

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


def recognized_face(data):
    global markers_pub, detect_true, sound_sent, color, det_entry, faces_count, face_marker, search_name, status, cancel_pub

    name = data.data
    #print name

    #when we have the position, but looking for the rotation (TO-DO: wait for the detection first)
    if status == 1 and search_name == name:
        cancel_pub.publish("cancel")
    
    thresh = 15
    thresh_reached = False
    
    if detect_true == 1 and det_entry['name'] == None:
        if name in faces_count:
            faces_count[name]['count'] += 1
            #print faces_count[name]['name']+": "+str(faces_count[name]['count'])
            if faces_count[name]['count'] > thresh:
                det_entry['name'] = name
                print "adding name to the dictionary!: ", det_entry['name'], det_entry['point']
                # find the closest node in graph
        else:
            faces_count[name] = {'count': 1, 'face': False, 'name': name}
    

def detection_thresh(points):
    global det, detected, detect_true, sound_sent, det_entry, faces_count, detect_again

    for newPoint in points.markers:
        position = newPoint.pose.position
        xp = round(position.x, 2)
        yp = round(position.y, 2)
        zp = round(position.z, 2)
        mind = 0.5
        min_point = None
        if zp > 1:
            print xp, yp, zp
            continue
        for key in det:
            (x, y, z) = key.split(';')
            dist_x = abs(xp - float(x))
            dist_y = abs(yp - float(y))
            dist_z = abs(zp - float(z))
            if dist_x <= mind and dist_y <= mind and dist_z <= mind:
                if not det[key]['detected']:
                    min_point = det[key]
                else:
                    min_point = "not"

        if min_point == None:
            detect_true = 0     #when we detect new face, we dont want recognizer to work immediately, but wait for this to become 1 (which means: face detected!!)
            det[str(xp) + ";" + str(yp) + ";" + str(zp)] = {'count': 1, 'detected': False, 'name': None, 'marker': None, 'face': True, 'point': str(xp) + ";" + str(yp) + ";" + str(zp)}
        elif min_point != "not":
            min_point['count'] += 1
            if min_point['count'] > 25:
                min_point['detected'] = True
                min_point['marker'] = newPoint      #we save the last marker
                detected += 1
                print "face " + str(detected) + " detected!!!!\n"
                #print det

                #so we detected a new face - let's reset the counters and ask recognizer what he sees
                detect_true = 1     #we are indeed looking at the face - let's allow recognizer to start counting!
                det_entry = min_point       # global variable, will save name of the person to it once the point is recognized
                sound_sent = 0
                for key1 in faces_count:            #reset to zero
                    faces_count[key1]['count'] = 0
                

        else:
            pass


def sign_detection(points):
    #the same as for the face detection
    global detect_sign_true, det_signs, sign_detected, det_entry_sign, signs_count, det, sign_detected_again

    for newPoint in points.markers:
        position = newPoint.pose.position
        xp = round(position.x, 2)
        yp = round(position.y, 2)
        zp = round(position.z, 2)
        mind = 0.5
        min_point = None
        if zp > 1:
            print xp, yp, zp
            continue
        for key in det:
            (x, y, z) = key.split(';')
            dist_x = abs(xp - float(x))
            dist_y = abs(yp - float(y))
            dist_z = abs(zp - float(z))
            if dist_x <= mind and dist_y <= mind and dist_z <= mind:
                if not det[key]['detected']:
                    min_point = det[key]
                else:
                    # we already have that point detected, lets count if it's really that point
                    min_point = "not"
                    sign_detected_again += 1

        if min_point == None:
            detect_sign_true = 0     #when we detect new sign, we dont want recognizer to work immediately, but wait for this to become 1 (which means: sign detected!!)
            det[str(xp) + ";" + str(yp) + ";" + str(zp)] = {'count': 1, 'detected': False, 'name': None, 'marker': None, 'face': False, 'point': str(xp) + ";" + str(yp) + ";" + str(zp)}
        elif min_point != "not":
            min_point['count'] += 1
            if min_point['count'] > 25:
                min_point['detected'] = True
                min_point['marker'] = newPoint      #we save the last marker
                sign_detected += 1
                print "sign" + str(detected) + " detected!!!!\n"
                #print det
                
                #so we detected a new sign - let's reset the counters and ask recognizer what he sees
                detect_sign_true = 1     #we are indeed looking at the face - let's allow recognizer to start counting!
                det_entry_sign = min_point       # global variable, will save name of the person to it once the point is recognized
                for key1 in signs_count:            #reset to zero
                    signs_count[key1]['count'] = 0

        else:
            pass

def recognized_sign(data):
    global signs_count
    global detect_sign_true
    global det_entry_sign

    name = data.data
    #print name

    thresh = 15
    thresh_reached = False
    
    if detect_sign_true == 1 and det_entry_sign['name'] == None:
        if name in  signs_count:
            signs_count[name]['count'] += 1
            #print signs_count[name]['name']+": "+str(signs_count[name]['count'])
            if signs_count[name]['count'] > thresh:
                det_entry_sign['name'] = name
                print "adding name to the dictionary!: ", det_entry_sign['name'], det_entry_sign['point']
                
                # we publish to the appropriate topic
                
        else:
            signs_count[name] = {'count': 1, 'face': False, 'name': name}

    
def voice_action(data):
    global search_name

    #print "You said: "+data.data

    #now do sth with that data!
    names = ['peter', 'harry', 'tina', 'scarlett', 'forest', 'kim', 'filip', 'mathew', 'ellen']
    objects = ['building']
    streets = ['street']
    colours = ['blue', 'yellow', 'green', 'red']
    sentence = data.data

    tokens = nltk.word_tokenize(sentence)
    #print "Tokens:\n"
    #print tokens

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
    #speak_robot("Where would you like to go, " + name)
    street_pub.publish(colour_street)


def publish_faces(non):
    global det, markers_pub, search_name, goto_pub, cancel_pub, status
    
    markers = []
    marker = Marker()

    for key in det:
        if det[key]['name'] == None:
            continue

        name = det[key]['name']
        marker = det[key]['marker']

        #lets check if we already have the face we are looking for (status = 0 means we are looking for a face)
        if search_name != None and status == 0:
            print "We have a search name and status is 0."
            if name == search_name:
                print "Name we are looking for is the same as the name in our dict. Sending directions."
                send_pose = marker.pose
                cancel_pub.publish("cancel")
                goto_pub.publish(send_pose)
                status = 1

        if name in color_map:
            marker.color = color_map[name]
        else:
            marker.color = ColorRGBA(0, 0, 0, 1)

        markers.append(marker)
        
    markers_pub.publish(markers)


def face_recognizer():
    global markers_pub, voice_pub, cancel_pub, street_pub, goto_pub, slow_pub, stop_pub, honk_pub, oneway_pub

    rospy.init_node('face_recognizer', anonymous=True)
    markers_pub = rospy.Publisher('/viz/markers', MarkerArray, queue_size=100)
    cancel_pub = rospy.Publisher('/cancel', String, queue_size=100)
    street_pub = rospy.Publisher('/search/street', String, queue_size=100)
    goto_pub = rospy.Publisher('/goto', Pose, queue_size=100)
    slow_pub = rospy.Publisher("/sign/slow", String, queue_size=100)
    stop_pub = rospy.Publisher("/sign/stop", String, queue_size=100)
    honk_pub = rospy.Publisher("/sign/honk", String, queue_size=100)
    oneway_pub = rospy.Publisher("/sign/oneway", Pose, queue_size=100)
    voice_pub = rospy.Publisher('/robotsound', SoundRequest, queue_size=100)
    rospy.Subscriber('/transformedMarkers/faces', MarkerArray, detection_thresh)
    rospy.Subscriber('/transformedMarkers/signs', MarkerArray, sign_detection)
    rospy.Subscriber('/recognizer/signs', String, recognized_sign)
    rospy.Subscriber('/recognizer/face', String, recognized_face)
    rospy.Subscriber('/tf', tf.msg.tfMessage, publish_faces)
    rospy.Subscriber("/command", String, voice_action)
    rospy.spin()

if __name__ == '__main__':
    face_recognizer()
