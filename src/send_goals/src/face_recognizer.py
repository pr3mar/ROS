#!/usr/bin/env python

import nltk
from nltk.metrics import edit_distance
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from std_msgs.msg import String, ColorRGBA
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import *
from visualization_msgs.msg import Marker, MarkerArray
from sound_play.msg import SoundRequest
import tf

# main status of app (which stage are we at at the moment)
status = 0      # 0 = looking for the face, 1 = approaching the face, 2 = approached the face, 3 = waiting for the command from the face

voice_pub = None
markers_pub = None
cancel_pub = None
goto_pub = None
goto_street_pub = None
slow_pub = None 
stop_pub = None
honk_pub = None
oneway_pub = None
det = {}
search_name = None
person_name = None
sent_street = 0
colour_street = None
alib = None

# faces
detected_faces = 0

# signs
detected_signs = 0

# honk: light blue, left: dark blue, limit: pink, oneway: light red, stop: dark red, peter: light green, tina: dark green, harry: light orange,  forest: orange, filip: brown, kim: tirquoise, mathew: purple, scarlett: light green, ellen: dark green-blue 
color_map = {'honk': ColorRGBA(85, 96, 240, 1), 'left': ColorRGBA(16, 31, 239, 1), 'limit': ColorRGBA(249, 98, 98, 1), 'oneway': ColorRGBA(245, 1, 1, 1), 'stop': ColorRGBA(152, 6, 6, 1), 'peter': ColorRGBA(128, 255, 0, 1), 'tina': ColorRGBA(51, 51, 255, 1), 'harry': ColorRGBA(242, 208, 15, 1), 'forest': ColorRGBA(255, 128, 0, 1), 'filip': ColorRGBA(102, 51, 0, 1), 'kim': ColorRGBA(19, 237, 226, 1), 'mathew': ColorRGBA(203, 17, 240, 1), 'scarlett': ColorRGBA(111, 255, 0, 1), 'ellen': ColorRGBA(10, 107, 107, 1)}


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



def face_detection(points):
    global det, detected_faces

    for newPoint in points.markers:
        position = newPoint.pose.position
        xp = round(position.x, 2)
        yp = round(position.y, 2)
        zp = round(position.z, 2)
        mind = 0.5
        min_point = None
        if zp > 0.6:
            print "false face: ", xp, yp, zp
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
            d = dict()
            det[str(xp) + ";" + str(yp) + ";" + str(zp)] = {'count': 1, 'max_recognition': 0, 'detected': False, 'name': None, 'name_dict': d, 'marker': None, 'face': True, 'point': str(xp) + ";" + str(yp) + ";" + str(zp)}
        elif min_point != "not":
            min_point['count'] += 1
            if min_point['count'] > 10:
                min_point['detected'] = True
                min_point['marker'] = newPoint      #we save the last marker
                print "face " + str(detected_faces) + " detected!!!!\n", min_point['point']
                detected_faces += 1
                #print det
        else:
            pass


def recognized_face(data):
    global det, search_name, status, cancel_pub, alib
    
    thresh = 15
    rec = data.data
    #print rec
    
    #when we have the position, but looking for the rotation
    #if status == 2 and search_name == name:
    #   goal = GoalID()
    #    cancel_pub.publish(goal)
    #    alib.cancel_goal()
    #    speak_robot(search_name + ", where would you like to go?")
    #    status = 3
    
    x, y, z, name = rec.split(';')
    xp = round(float(x), 2)
    yp = round(float(y), 2)
    zp = round(float(z), 2)
    mind = 0.5
    for key in det:
        (x, y, z) = key.split(';')
        dist_x = abs(xp - float(x))
        dist_y = abs(yp - float(y))
        dist_z = abs(zp - float(z))
        if dist_x <= mind and dist_y <= mind and dist_z <= mind:
            #so we have a recognition that maches given position of a detection -> let's store the recognized name
            det[key]['name_dict'][name] = det[key]['name_dict'].get(name, 1) + 1    #we increase a count, or if the key doesn't exist yet, get() allows us to set default value
            #print name, "count:", det[key]['name_dict'][name]
            
            if det[key]['name_dict'][name] > det[key]['max_recognition']:     #we check if current name count is bigger than max count of all names
                det[key]['max_recognition'] = det[key]['name_dict'][name]
            
            if det[key]['name_dict'][name] > thresh and det[key]['name_dict'][name] == det[key]['max_recognition'] and det[key]['name'] != name :    #if we dont want the recognized name to be changed once set, add to IF: and det[key]['name'] == None 
                #if count of a recognized name is now bigger than threshold, and is also the same as "max_recognition", which means its a max of all names, we save the name
                det[key]['name'] = name
                print "Adding name to the face detection ", det[key]['point'], ": ", name
    


def sign_detection(points):
    global det, detected_signs

    for newPoint in points.markers:
        position = newPoint.pose.position
        xp = round(position.x, 2)
        yp = round(position.y, 2)
        zp = round(position.z, 2)
        mind = 0.5
        min_point = None
        if zp > 0.6:
            print "false sign: ", xp, yp, zp
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
            d = dict()
            det[str(xp) + ";" + str(yp) + ";" + str(zp)] = {'count': 1, 'max_recognition': 0, 'detected': False, 'name': None, 'name_dict': d, 'marker': None, 'face': False, 'point': str(xp) + ";" + str(yp) + ";" + str(zp)}
        elif min_point != "not":
            min_point['count'] += 1
            if min_point['count'] > 10:
                min_point['detected'] = True
                min_point['marker'] = newPoint      #we save the last marker
                print "Sign " + str(detected_signs) + " detected!!!!\n", min_point['point']
                detected_signs += 1
                #print det
        else:
            pass



def recognized_sign(data):
    global det, stop_pub, slow_pub, oneway_pub
    
    thresh = 15
    rec = data.data
    #print rec
    
    x, y, z, name = rec.split(';')
    xp = round(float(x), 2)
    yp = round(float(y), 2)
    zp = round(float(z), 2)
    mind = 0.5
    for key in det:
        (x, y, z) = key.split(';')
        dist_x = abs(xp - float(x))
        dist_y = abs(yp - float(y))
        dist_z = abs(zp - float(z))
        if dist_x <= mind and dist_y <= mind and dist_z <= mind:
            #so we have a recognition that maches given position of a detection -> let's store the recognized sign
            det[key]['name_dict'][name] = det[key]['name_dict'].get(name, 1) + 1    #we increase a count, or if the key doesn't exist yet, get() allows us to set default value
            #print name, "count:", det[key]['name_dict'][name]
            
            if det[key]['name_dict'][name] > det[key]['max_recognition']:     #we check if current name count is bigger than max count of all names
                det[key]['max_recognition'] = det[key]['name_dict'][name]
            
            if det[key]['name_dict'][name] > thresh and det[key]['name'] == None :    #if we dont want the recognized name to be changed once set, add to IF: and det[key]['name'] == None 
                #if count of a recognized name is now bigger than threshold, and is also the same as "max_recognition", which means its a max of all names, we save the name
                det[key]['name'] = name
                print "Adding name to the sign detection ", det[key]['point'], ": ", name
                
                # we publish to the appropriate topic ->> TO-DO: this should get moved!!
                if name == 'honk':
                    #honk_pub.publish("true")
                    speak_robot("Beeeeeeeeeeeeeeeeeeeeep! Bip bip!")
                elif name =='stop':
                    stop_pub.publish("true")
                elif name =='limit':   
                    slow_pub.publish("true")  
                elif name =='oneway':  
                    oneway_pub.publish(det_entry_sign['marker'].pose)
    


def voice_action(data):
    global search_name, colour_street, street_pub, sent_street, status, cancel_pub, det

    #status = -1
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
    #street_pub.publish(colour_street)
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

def publish_faces(non):
    global det, markers_pub, search_name, goto_pub, cancel_pub, status, colour_street, sent_street, street_pub, alib
    
    markers = []
    marker = Marker()

    for key in det:
        if det[key]['name'] == None or det[key]['detected'] == False:
            continue

        name = det[key]['name']
        marker = det[key]['marker']
        marker.type = 9
        marker.text = name
        #print "coloring marker: ", name, color_map[name]
        #if name in color_map:
            #print "custom color"
            #marker.color = color_map[name]
            #marker.color.r = marker.color.r / 255.0
            #marker.color.g = marker.color.g / 255.0
            #marker.color.b = marker.color.b / 255.0
        #else:
            #print "default color"
            #marker.color = ColorRGBA(0, 0, 0, 1)

        #print "marker color: " + marker.color
        markers.append(marker)

        #lets check if we already have the face we are looking for (status = 0 means we are looking for a face)
        if search_name != None and status == 0:
            #print "We have a search name and status is 0."
            if name == search_name:
                sent_street = 1     #we don't have to send street color, if we know where the face is
                status = 1
                print "Name we are looking for is the same as the name in our dict. Sending directions. name we are looking for: ", search_name, "Name in our dict: ", name 
                send_pose = marker.pose
                goal = GoalID()
                cancel_pub.publish(goal)
                #alib.cancel_goal()
                print "canceling in loop, waiting"
                time = rospy.get_time() + rospy.Duration(1.0).to_sec()
                while(time > rospy.get_time()):
			        #print "time = ", time, "time now = ", rospy.get_time() 
			        pass
	                
                goto_pub.publish(send_pose)
                

    markers_pub.publish(markers)
    
    if sent_street == 0 and colour_street != None and status == 0:
        sent_street = 1
        print "sending goal!"
        goal = GoalID()
        cancel_pub.publish(goal)
        #alib.cancelGoal()
        print "canceling previous goal! waiting:"
        time = rospy.get_time() + rospy.Duration(1.0).to_sec()
        while(time > rospy.get_time()):
            #print "time = ", time, "time now = ", rospy.get_time() 
            pass
        
        print "sending: "
        street_pub.publish(colour_street)
        

def goal_reached(data):
    global status, search_name, person_name

    print "face reached!"
    #when we have the position, but looking for the rotation (TO-DO: wait for the detection first)
    if data.data == 'true' and status == 1:
        status = 2
        person_name = search_name
        speak_robot(person_name + ", where would you like to go?")
        print person_name, " where would you like to go?"
        status = 3

    elif data.data == 'false' and status == 1:
        speak_robot("Dispatcher, you said "+person_name+" is waiting on the "+colour_street + " street, right?")
        print person_name, colour_street, " Where is he?"
        status = -1    

    elif data.data == 'true' and status == 3:
        speak_robot(person_name + ", we have reached your destination!")
        print person_name, " we have reached your destination!"
        status = 4


def face_recognizer():
    global markers_pub, voice_pub, cancel_pub, street_pub, goto_pub, slow_pub, stop_pub, honk_pub, oneway_pub, alib, goto_street_pub

    alib = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.init_node('face_recognizer', anonymous=True)
    markers_pub = rospy.Publisher('/viz/markers', MarkerArray, queue_size=100)
    cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=100)
    street_pub = rospy.Publisher('/search/street', String, queue_size=100)
    goto_pub = rospy.Publisher('/goto/pose', Pose, queue_size=100)
    goto_street_pub = rospy.Publisher('/goto/street', String, queue_size=100)
    slow_pub = rospy.Publisher('/sign/slow', String, queue_size=100)
    stop_pub = rospy.Publisher('/sign/stop', String, queue_size=100)
    honk_pub = rospy.Publisher('/sign/honk', String, queue_size=100)
    oneway_pub = rospy.Publisher('/sign/oneway', Pose, queue_size=100)
    voice_pub = rospy.Publisher('/robotsound', SoundRequest, queue_size=100)
    rospy.Subscriber('/transformedMarkers/faces', MarkerArray, face_detection)
    rospy.Subscriber('/transformedMarkers/signs', MarkerArray, sign_detection)
    #rospy.Subscriber('/recognizer/signs', String, recognized_sign)
    rospy.Subscriber('/recognizer/face', String, recognized_face)
    rospy.Subscriber('/tf', tf.msg.tfMessage, publish_faces)
    rospy.Subscriber('/command', String, voice_action)
    rospy.Subscriber('/goal_reached', String, goal_reached)
    rospy.spin()

if __name__ == '__main__':
    face_recognizer()
