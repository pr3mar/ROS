#!/usr/bin/env python

import roslib, sys
import rospy
import actionlib
import random
import math
import priorityDictionary

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
		self.defineEdges(edges)
		self.printGraph()

	# get a list representing the nearest path
	def dijkstra(self, fr, to):
		pass

if __name__ == '__main__':
	try:
		goals = []
		for i in range(1,5):
			goal = MoveBaseGoal()
			goal.target_pose.pose.position.x = i * 3
			goal.target_pose.pose.position.y = 2 * i
			goals.append(goal)
		edges = [(0, 1), (0, 2), (1, 2), (3, 1)]
		graph = Graph(goals, edges)
		pose = Pose()
		pose.position.x = 7.56917
		pose.position.y = 4.91049
		ret = graph.localize(pose)
		print "nearest to: ", ret[0], "distance = ", ret[1]
	except rospy.ROSInterruptException:
		pass
