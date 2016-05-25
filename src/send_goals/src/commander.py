#!/usr/bin/env python
import nltk
from nltk.metrics import edit_distance
import rospy
from std_msgs.msg import String
from sound_play.msg import SoundRequest


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



def callback(data):
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



    #this doesnt work the first time
    tmp = SoundRequest()
    tmp.sound = -3
    tmp.command = 1
    tmp.arg = name + ", where would you like to go?"
    tmp.arg2 = ''
    
    pub = rospy.Publisher('/robotsound', SoundRequest, queue_size=1)
    pub.publish(tmp)
    
    
def commander():

    rospy.init_node('commander', anonymous=True)
    rospy.Subscriber("/command", String, callback)
    
    
    rospy.spin()

if __name__ == '__main__':
    commander()
