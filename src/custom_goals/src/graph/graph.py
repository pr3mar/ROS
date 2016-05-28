#!/usr/bin/env python

import roslib, sys
import rospy
import actionlib
import random

# from sensor_msgs.msg import image_encodings # do I need this?
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import *

#Edge class presents an edge/arc/connection in the graph
class Edge:
	def calcDist(self):
		x1 = self.fr.goal.target_pose.pose.position.x
		y1 = self.fr.goal.target_pose.pose.position.x
		x2 = self.to.goal.target_pose.pose.position.y
		y2 = self.to.goal.target_pose.pose.position.y
		return 

	def __init__(self, fr, to):
		self.fr = fr
		self.to = to
		self.cost = calcDist()

#Node class presents a node in the graph
class Node:
	node = 0
	def addInEdge(self, newIn):
		if not(newIn in self.ingoing):
			self.ingoing.add(newIn)
			newIn.addOutEdge(self)

	def addOutEdge(self, newOut):
		if not(newOut in self.outgoing):
			self.outgoing.add(newOut)
			newOut.addInEdge(self)

	def getDegree(self):
		return (len(self.ingoing) + len(self.outgoing) )

	def __init__(self, goal):
		self.goal = goal
		self.ingoing = set()
		self.outgoing = set()
		self.id = Node.node
		Node.node += 1

#main graph class
class Graph:
	def printGraph(self):
		for el in self.nodes:
			print el.id, el.getDegree()
			for x in el.ingoing:
				print x,
			print
			for x in el.outgoing:
				print x
			print
	def __init__(self, goals):
		self.nodes = []
		for goal in goals:
			self.nodes.append(Node(goal))
		for node in self.nodes:
			node.addInEdge(random.choice(self.nodes))
		self.printGraph()
	# get a list representing the nearest path
	def dijkstra(self, fr, to):
		pass

if __name__ == '__main__':
	try:
		goals = []
		for i in range(1,10):
			goal = MoveBaseGoal()
			goal.target_pose.pose.position.x = i * 3
			goal.target_pose.pose.position.x = 2 * i
			goals.append(goal)
		graph = Graph(goals)
	except rospy.ROSInterruptException:
		pass