#!/usr/bin/env python

import rospy
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import *
from visualization_msgs.msg import Marker, MarkerArray

markers_pub = None
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


def recognized_face(data):
    global peter
    global tina
    global harry
    global forest
    global filip
    global kim
    global matthew
    global scarlett
    global ellen
    global current_point
    global markers_pub
    global detect_true

    name = data.data
    print name
    
    markers = []
    marker = Marker()
    marker = current_point
    thresh = 50
    
    if detect_true == 1:
        if name == 'peter':
            peter += 1
            print peter
            if peter > thresh:
                print "publishing marker!"
                marker.color = ColorRGBA(128, 255, 0, 1)
                markers.append(marker)
                markers_pub.publish(markers)

        elif name == 'tina':
            tina += 1
            print tina
            if tina > thresh:
                marker.color = ColorRGBA(51, 51, 255, 1)
                markers.append(marker)
                markers_pub.publish(markers)
                
        elif name == 'harry':
            harry += 1
            print harry
            if harry > thresh:
                marker.color = ColorRGBA(255, 0, 0, 1)
                markers.append(marker)
                markers_pub.publish(markers)
        
        elif name == 'forest':
            forest += 1
            print forest
            if forest > thresh:
                marker.color = ColorRGBA(255, 128, 0, 1)
                markers.append(marker)
                markers_pub.publish(markers)
                
        elif name == 'filip':
            filip += 1
            print filip
            if filip > thresh:
                marker.color = ColorRGBA(102, 51, 0, 1)
                markers.append(marker)
                markers_pub.publish(markers)
                
        elif name == 'kim':
            kim += 1
            print kim
            if kim > thresh:
                marker.color = ColorRGBA(153, 0, 153, 1)
                markers.append(marker)
                markers_pub.publish(markers)
                
        elif name == 'matthew':
            matthew += 1
            print matthew
            if matthew > thresh:
                marker.color = ColorRGBA(255, 255, 0, 1)
                markers.append(marker)
                markers_pub.publish(markers)
                
        elif name == 'scarlett':
            scarlett += 1
            print scarlett
            if scarlett > thresh:
                marker.color = ColorRGBA(255, 0, 127, 1)
                markers.append(marker)
                markers_pub.publish(markers)
                
        elif name == 'ellen':
            ellen += 1
            print ellen
            if ellen > thresh:
                marker.color = ColorRGBA(0, 255, 255, 1)
                markers.append(marker)
                markers_pub.publish(markers)
    
    

def detection_thresh(points):
    global peter
    global tina
    global harry
    global forest
    global filip
    global kim
    global matthew
    global scarlett
    global ellen
    global current_point
    global det
    global detected
    global detect_true

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
            det[str(xp) + ";" + str(yp) + ";" + str(zp)] = {'count': 1, 'detected': False,
                                                                 'point': str(xp) + ";" + str(yp) + ";" + str(zp)}
        elif min_point != "not":
            min_point['count'] += 1
            if min_point['count'] > 25:
                min_point['detected'] = True
                detected += 1
                print "face " + str(detected) + " detected!!!!\n" + str(min_point)
                print det

                #so we detected a face - let's reset the counters and ask recognizer what he sees
                detect_true = 1
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
    
def face_recognizer():
    global markers_pub
    rospy.init_node('face_recognizer', anonymous=True)
    rospy.Subscriber('facedetector/markers', MarkerArray, detection_thresh)
    markers_pub = rospy.Publisher('recognizer/face_marker', MarkerArray)
    rospy.Subscriber('face_recognizer/face', String, recognized_face)

    rospy.spin()

if __name__ == '__main__':
    face_recognizer()
