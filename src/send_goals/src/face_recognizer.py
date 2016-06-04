#!/usr/bin/env python

import nltk
from nltk.metrics import edit_distance
import rospy
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import *
from visualization_msgs.msg import Marker, MarkerArray
from sound_play.msg import SoundRequest

voice_pub = None
markers_pub = None

#faces
current_point = None
det = {}
detected = 0
detect_true = 0
peter = 0
tina = 0
harry = 0
forest = 0
filip = 0
kim = 0
matthew = 0
scarlett = 0
ellen = 0
sound_sent = 0
color = None
det_entry = None

#signs
det_signs = {}
sign_detected = 0
detect_sign_true = 0
det_entry_sign = None
all_things_detected = {}


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

    print "Name we are looking for: %s. It is similar to: %s"%(name, original)
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

    print "Name we are looking for: %s. It is similar to: %s"%(new, original)
    return  new


def speak_robot(str_speech):
    #this doesnt work the first time - nobody knows why???
    print str_speech
    tmp = SoundRequest()
    tmp.sound = -3
    tmp.command = 1
    tmp.arg = str_speech
    tmp.arg2 = ''

    global voice_pub
    voice_pub.publish(tmp)


def recognized_face(data):
    global peter, tina, harry, forest, filip, kim, matthew, scarlett, ellen, current_point, markers_pub, detect_true, sound_sent, color, det_entry

    name = data.data
    print name
    
    markers = []
    marker = Marker()
    marker = current_point
    thresh = 15
    thresh_reached = False
    
    # TO-DO: stop voting once the thresh is reached!
    
    if detect_true == 1:
        if name == 'peter':
            peter += 1
            print peter
            if peter > thresh:
                print "Recognized Peter!"
                if sound_sent == 0:
                    sound_sent = 1
                    speak_robot("We found Peter!")
                
                det_entry['name'] = name
                marker.color = ColorRGBA(128, 255, 0, 1)
                markers.append(marker)
                markers_pub.publish(markers)

        elif name == 'tina':
            tina += 1
            print tina
            if tina > thresh:
                print "Recognized Tina!"
                if sound_sent == 0:
                    sound_sent = 1
                    speak_robot("We found Tina!")
                det_entry['name'] = name    
                marker.color = ColorRGBA(51, 51, 255, 1)
                markers.append(marker)
                markers_pub.publish(markers)
                
        elif name == 'harry':
            harry += 1
            print harry
            if harry > thresh:
                if sound_sent == 0:
                    sound_sent = 1
                    speak_robot("We found Harry Potter, the boy who lived!")
                det_entry['name'] = name    
                marker.color = ColorRGBA(255, 0, 0, 1)
                markers.append(marker)
                markers_pub.publish(markers)
        
        elif name == 'forest':
            forest += 1
            print forest
            if forest > thresh:
                if sound_sent == 0:
                    sound_sent = 1
                    speak_robot("We found Forest Gump, and of course he was running!")
                det_entry['name'] = name
                marker.color = ColorRGBA(255, 128, 0, 1)
                markers.append(marker)
                markers_pub.publish(markers)
                
        elif name == 'filip':
            filip += 1
            print filip
            if filip > thresh:
                if sound_sent == 0:
                    sound_sent = 1
                    speak_robot("We found Fillip!")
                det_entry['name'] = name
                marker.color = ColorRGBA(102, 51, 0, 1)
                markers.append(marker)
                markers_pub.publish(markers)
                
        elif name == 'kim':
            kim += 1
            print kim
            if kim > thresh:
                if sound_sent == 0:
                    sound_sent = 1
                    speak_robot("We found Kim! Just kidding, no one finds Kim.")
                det_entry['name'] = name    
                marker.color = ColorRGBA(153, 0, 153, 1)
                markers.append(marker)
                markers_pub.publish(markers)
                
        elif name == 'mathew':
            matthew += 1
            print matthew
            if matthew > thresh:
                if sound_sent == 0:
                    sound_sent = 1
                    speak_robot("We found Matthew!")
                det_entry['name'] = name
                marker.color = ColorRGBA(255, 255, 0, 1)
                markers.append(marker)
                markers_pub.publish(markers)
                
        elif name == 'scarlett':
            scarlett += 1
            print scarlett
            if scarlett > thresh:
                if sound_sent == 0:
                    sound_sent = 1
                    speak_robot("We found Scarlett!")
                det_entry['name'] = name
                marker.color = ColorRGBA(255, 0, 127, 1)
                markers.append(marker)
                markers_pub.publish(markers)
                
        elif name == 'ellen':
            ellen += 1
            print ellen
            if ellen > thresh:
                if sound_sent == 0:
                    sound_sent = 1
                    speak_robot("We found Ellen and it was quite a show!")
                det_entry['name'] = name
                marker.color = ColorRGBA(0, 255, 255, 1)
                markers.append(marker)
                markers_pub.publish(markers)
    
    

def detection_thresh(points):
    global peter, tina, harry, forest, filip, kim, matthew, scarlett, ellen, current_point, det, detected, detect_true, sound_sent, det_entry

    for newPoint in points.markers:
        current_point = newPoint
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
            det[str(xp) + ";" + str(yp) + ";" + str(zp)] = {'count': 1, 'detected': False, 'name': None, 'point': str(xp) + ";" + str(yp) + ";" + str(zp)}
        elif min_point != "not":
            min_point['count'] += 1
            if min_point['count'] > 25:
                min_point['detected'] = True
                detected += 1
                print "face " + str(detected) + " detected!!!!\n" + str(min_point)
                print det

                #so we detected a new face - let's reset the counters and ask recognizer what he sees
                detect_true = 1     #we are indeed looking at the face - let's allow recognizer to start counting!
                det_entry = min_point       # global variable, will save name of the person to it once the point is recognized
                sound_sent = 0
                peter = 0
                tina = 0
                harry = 0
                forest = 0
                filip = 0
                kim = 0
                matthew = 0
                scarlett = 0
                ellen = 0
                

        else:
            pass


def sign_detection(points):
    #the same as for the face detection
    global detect_sign_true
    global det_signs
    global sign_detected
    global det_entry_sign
    global all_things_detected
    global det

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
            detect_sign_true = 0     #when we detect new sign, we dont want recognizer to work immediately, but wait for this to become 1 (which means: sign detected!!)
            det[str(xp) + ";" + str(yp) + ";" + str(zp)] = {'count': 1, 'detected': False, 'name': None, 'point': str(xp) + ";" + str(yp) + ";" + str(zp)}
        elif min_point != "not":
            min_point['count'] += 1
            if min_point['count'] > 25:
                min_point['detected'] = True
                sign_detected += 1
                print "sign" + str(detected) + " detected!!!!\n" + str(min_point)
                print det
                
                #so we detected a new sign - let's reset the counters and ask recognizer what he sees
                detect_sign_true = 1     #we are indeed looking at the face - let's allow recognizer to start counting!
                det_entry_sign = min_point       # global variable, will save name of the person to it once the point is recognized
                for key1 in all_things_detected:            #reset to zero
                    all_things_detected[key1][count] = 0

        else:
            pass

def recognized_sign(data):
    global all_things_detected
    global detect_sign_true
    global det_entry_sign

    name = data.data
    print name

    thresh = 15
    thresh_reached = False
    
    if detect_sign_true == 1 and det_entry_sign['name'] == None:
        if name in  all_things_detected:
            all_things_detected[name][count] += 1
            print all_things_detected[name][name]+": "+all_things_detected[name][count]
            if all_things_detected[name][count] > thresh:
                det_entry_sign['name'] = name
                print "adding to the dictionary!"
                # find the closest node in graph
        else:
            all_things_detected[name] = {'count': 1, 'face': False, 'name': name}



def voice_action(data):
    print "You said: "+data.data

    #now do sth with that data!
    names = ['Peter', 'Harry', 'Tina', 'Scarlet', 'Forest', 'Kim', 'Filip', 'Matthew', 'Ellen']
    objects = ['building']
    streets = ['street']
    colours = ['blue', 'yellow', 'green', 'red']
    sentence = data.data

    tokens = nltk.word_tokenize(sentence)
    print "Tokens:\n"
    print tokens

    #first word is name
    colour_building = None
    colour_street = None
    name = min_distance_one(tokens[0], names)

    building = min_distance_all(tokens, objects)
    if building[0] != None:
        colour_building = min_distance_one(tokens[building[1]-1], colours)

    street = min_distance_all(tokens, streets)
    if street[0] != None:
        colour_street = min_distance_one(tokens[street[1]-1], colours)

    print "Mission Impossible: %s %s %s %s %s"%(name,  colour_building, building[0], colour_street, street[0])
    speak_robot("Where would you like to go, " + name)

def publish_faces():
    global det
    global markers_pub

    markers = []
    marker = Marker()

    for key in det:
        (x, y, z) = key.split(';')



def face_recognizer():
    global markers_pub
    global voice_pub
    rospy.init_node('face_recognizer', anonymous=True)
    markers_pub = rospy.Publisher('recognizer/face_markers', MarkerArray)
    voice_pub = rospy.Publisher('/robotsound', SoundRequest, queue_size=1)
    rospy.Subscriber('pcl/transformedMarkers', MarkerArray, detection_thresh)
    rospy.Subscriber('/signdetector/markers', MarkerArray, sign_detection)
    rospy.Subscriber('face_recognizer/face', String, recognized_face)
    rospy.Subscriber("/command", String, voice_action)
    rospy.spin()

if __name__ == '__main__':
    face_recognizer()
