#!/usr/bin/env python

import roslib, sys
import rospy
import actionlib
import random
import math
import priorityDictionary
from Queue import PriorityQueue

# from sensor_msgs.msg import image_encodings # do I need this?
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import *

#Edge class presents an edge/arc/connection in the graph
class Edge:
	def calcDist(self):
		x1 = self.fr.goal.target_pose.pose.position.x
		y1 = self.fr.goal.target_pose.pose.position.y
		x2 = self.to.goal.target_pose.pose.position.x
		y2 = self.to.goal.target_pose.pose.position.y
		# print x1, y1, "; ", x2, y2, "dist = ", math.sqrt( (x2 - x1)**2 + (y2 - y1)**2 )
		return math.sqrt( (x2 - x1)**2 + (y2 - y1)**2 )

	def __str__(self):
		return str("from: " + str(self.fr) + ", to: " + str(self.to))

	def __init__(self, fr, to, typ):
		self.fr = fr
		self.to = to
		self.type = typ # type = {true, false} -> True == ingoing, False == outgoing
		self.cost = self.calcDist()

#Node class presents a node in the graph
class Node:
	node = 0
	# def addInEdge(self, newIn):
	# 	if not(newIn in self.ingoing):
	# 		self.ingoing.add(newIn)
	# 		newIn.addOutEdge(self)

	# def addOutEdge(self, newOut):
	# 	if not(newOut in self.outgoing):
	# 		self.outgoing.add(newOut)
	# 		newOut.addInEdge(self)
	def check(self, node, typ):
		if typ:
			for el in self.ingoing:
				if(el.fr == node):
					return False
		else:
			for el in self.outgoing:
				if(el.to == node):
					return False
		return True


	def addInEdge(self, node):
		if(self.check(node, True)):
			self.ingoing.append(Edge(node, self, True))
			node.addOutEdge(self)

	def addOutEdge(self, node):
		if(self.check(node, False)):
			self.outgoing.append(Edge(self, node, False))
			node.addInEdge(self)

	def getDegree(self):
		return (len(self.ingoing) + len(self.outgoing) )

	def __init__(self, goal):
		self.goal = goal
		self.ingoing = []
		self.outgoing = []
		self.id = Node.node
		Node.node += 1

	def getCoords(self):
		return "x = " + str(self.goal.target_pose.pose.position.x) + \
					", y = " + str(self.goal.target_pose.pose.position.y) 


	def __str__(self):
		return str(self.id)

#main graph class
class Graph:
	def printGraph(self):
		for el in self.nodes:
			print "current:", el, ", has degree =", el.getDegree(), \
						", coords = "  + el.getCoords()
			print "ingoing  = ",
			for x in el.ingoing:
				print x, "(dist = ", x.cost,"); ",
			print "\noutgoing = ", 
			for x in el.outgoing:
				print x, "(dist = ", x.cost,"); ",
			print "\n"

	def getNodeByID(self, id):
		for node in self.nodes:
			if(node.id == id):
				return node
		return None

	def defineEdges(self, edges):
		for edge in edges:
			# print edge
			fr = self.getNodeByID(edge[0])
			to = self.getNodeByID(edge[1])
			if(fr == None or to == None):
				raise Exception("Funny thing, no such ID")
			to.addInEdge(fr)
			fr.addOutEdge(to)

	def calcDist(self, pose, node):
		x1 = pose.position.x
		y1 = pose.position.y
		x2 = node.goal.target_pose.pose.position.x
		y2 = node.goal.target_pose.pose.position.y
		# print x1, y1, "; ", x2, y2, "dist = ", math.sqrt( (x2 - x1)**2 + (y2 - y1)**2 )
		return math.sqrt( (x2 - x1) ** 2 + (y2 - y1) ** 2 )

	def localize(self, pose):
		minDist = sys.maxsize
		minNode = None
		for node in self.nodes:
			dist = self.calcDist(pose, node)
			if(dist < minDist):
				minDist = dist
				minNode = node
		return (minNode, minDist)

	def __init__(self, goals, edges):
		self.nodes = []
		for goal in goals:
			self.nodes.append(Node(goal))
		# for node in self.nodes:
			# node.addInEdge(random.choice(self.nodes))
		try:
			self.defineEdges(edges)
		except Exception as e:
			print e
		self.printGraph()

	def calcHeuristic(self, fr, to):
		x1 = fr.goal.target_pose.pose.position.x
		y1 = fr.goal.target_pose.pose.position.y
		x2 = to.goal.target_pose.pose.position.x
		y2 = to.goal.target_pose.pose.position.y
		# print x1, y1, "; ", x2, y2, "dist = ", math.sqrt( (x2 - x1)**2 + (y2 - y1)**2 )
		return math.sqrt( (x2 - x1)**2 + (y2 - y1)**2 )
	
	def minFScore(self, fScore, toVisit):
		# print "new minFScore"
		minF = sys.maxsize
		node = None

		for n in fScore:
			if not(n in toVisit): 
				# print "continue: ", n, 
				# self.printSet(toVisit)
				continue
			# print "scores:", fScore[n], minF
			if fScore[n] < minF:
				# print "set n = ", n
				node = n
				minF = fScore[n]
		return node

	def Astar(self, fr, to):
		visited = set()
		toVisit = set()
		toVisit.add(str(fr.id))
		path = {}
		# set the scores
		gScore = {}
		fScore = {}
		for node in self.nodes:
			gScore[str(node.id)] = sys.maxsize
			fScore[str(node.id)] = sys.maxsize
		# set first values
		gScore[str(fr.id)] = 0
		fScore[str(fr.id)] = self.calcHeuristic(fr, to)
		# start iterating
		iter = 0
		while len(toVisit):
			# get the minimal F-score
			current = self.minFScore(fScore, toVisit)
			# print current
			currentNode = self.getNodeByID(int(current))
			# debugging info
			if currentNode == None:
				print "none!!!!", current
				oidhgkjsdfg
			# print "iter = ", iter, "current = ", current
			# check if it the goal node, return the path
			if current == str(to.id):
				print "found path!!!"
				return self.reconstructPath(path, current)
		
			# remove from the queue
			toVisit.remove(current)
			# # debug info
			# self.printSet(toVisit)
			# print len(toVisit)
			# add to visited 
			visited.add(current)
			# iterate over outgoing edges
			for out in currentNode.outgoing:
				# get the node on the other side
				out = out.to
				# check if it was alredy visited
				if str(out.id) in visited:
					continue
				# get the score
				tentative_gScore = gScore[current] + self.calcHeuristic(currentNode, out)
				# print "tentative_gScore =", tentative_gScore
				# check if it is in the queue
				if not(out.id in toVisit):
					toVisit.add(str(out.id))
				# if the score is too high skip it
				elif tentative_gScore >= gScore[current]:
					continue
				# print "got here finally!"
				# get the real path
				path[str(out.id)] = current
				gScore[str(out.id)] = tentative_gScore
				fScore[str(out.id)] = gScore[str(out.id)] + self.calcHeuristic(out, to)
			# debug info
			iter += 1
			# self.printSet(toVisit)
		return None

	def reconstructPath(self, path, current):
		total_path = [current]
		while current in path:
			current = path[current]
			total_path.append(current)

		return [int(x) for x in reversed(total_path)]
	# util
	def printSet(self, s):
		print "set elements = [",
		for e in s:
			print e + ", ",
		print "]"

if __name__ == '__main__':
	try:
		goals = []
		#     0  1  2  3  4  5
		xx = [0,-2, 0, 2, 1,-1]
		yy = [0, 1, 1, 1, 2, 4]
		for i in range(0,6):
			goal = MoveBaseGoal()
			goal.target_pose.pose.position.x = xx[i]#random.randrange(0, 20)
			goal.target_pose.pose.position.y = yy[i]#random.randrange(0, 20)
			goals.append(goal)
		# edges = [(0, 1), (0, 2), (0, 4), (1, 5), (2, 3), (3,6), (4, 5), (5, 6), (5, 7), (6, 8), (7, 9), (8,9)]
		edges = [(0, 1), (0, 2), (0, 3), (1, 2), (1, 5), (2, 4), (3, 4), (4, 5)]
		graph = Graph(goals, edges)
		pose = Pose()
		pose.position.x = xx[0]
		pose.position.y = yy[0]
		p1 = graph.localize(pose)
		pose.position.x = xx[5]
		pose.position.y = yy[5]
		p2 = graph.localize(pose)
		print "nearest to: ", p1[0], "distance = ", p1[1]
		print "nearest to: ", p2[0], "distance = ", p2[1]
		print graph.Astar(p1[0], p2[0])
	except rospy.ROSInterruptException:
		pass
