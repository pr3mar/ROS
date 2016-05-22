#!/usr/bin/env python

import nltk
from nltk.metrics import edit_distance


def min_distance(tokens, find):
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
	return 	name, min_index


def min_colour(colour_original, find):
	#check the colour

	if colour_original == None:
		return None

	colour = None
	min_dist = 100
	for j in find:
		distance = edit_distance(colour_original, j, transpositions=False)
		if distance < min_dist:
			min_dist = distance
			colour = j

	print "Name we are looking for: %s. It is similar to: %s"%(colour, colour_original)
	return 	colour



names = ['Peter', 'Harry', 'Tina', 'Scarlet', 'Forest', 'Kim', 'Filip', 'Matthew', 'Ellen']
objects = ['building']
streets = ['street']
colours = ['blue', 'yellow', 'green', 'red']
sentence = "Pehar wants to go to the red stred to the bloo builder."


tokens = nltk.word_tokenize(sentence)
print "Tokens:\n"
print tokens

#first word is name
colour_building = None
colour_street = None
name = min_colour(tokens[0], names)

building = min_distance(tokens, objects)
if building[0] != None:
	colour_building = min_colour(tokens[building[1]-1], colours)

street = min_distance(tokens, streets)
if street[0] != None:
	colour_street = min_colour(tokens[street[1]-1], colours)

print "Mission Impossible: %s %s %s %s %s"%(name,  colour_building, building[0], colour_street, street[0])





#ADVANCED
tagged = nltk.pos_tag(tokens)
#print tagged
entities = nltk.chunk.ne_chunk(tagged)
#print entities

person_list = []
for subtree in entities.subtrees(filter=lambda t: t.label() == 'PERSON'):
        for leaf in subtree.leaves():
            person_list.append(leaf[0])


print person_list

name = None
min_dist = 100
for i in names:
	distance = edit_distance(i, person_list[0], transpositions=False)
	if distance < min_dist:
		min_dist = distance
		name = i

print name

