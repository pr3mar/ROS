#!/usr/bin/env python
import roslib, sys
import rospy
import actionlib
import random
import math

# from sensor_msgs.msg import image_encodings # do I need this?
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import *

#Edge class presents an edge/arc/connection in the graph
class Edge:
	# set edge cost
	def calcDist(self):
		x1 = self.fr.goal.target_pose.pose.position.x
		y1 = self.fr.goal.target_pose.pose.position.y
		x2 = self.to.goal.target_pose.pose.position.x
		y2 = self.to.goal.target_pose.pose.position.y
		# print x1, y1, "; ", x2, y2, "dist = ", math.sqrt( (x2 - x1)**2 + (y2 - y1)**2 )
		return math.sqrt( (x2 - x1)**2 + (y2 - y1)**2 )

	def __eq__(self, other):
		if isinstance(other, Edge):
			return self.to.id == other.to.id and self.fr.id == other.fr.id
		return NotImplemented
	
	# to String override
	def __str__(self):
		return str("from: " + str(self.fr) + ", to: " + str(self.to))

	# constructor
	def __init__(self, fr, to, typ):
		self.fr = fr
		self.to = to
		self.type = typ # type = {true, false} -> True == ingoing, False == outgoing
		self.cost = self.calcDist()

#Node class presents a node in the graph
class Node:
	# static variable to determine the ID of each node of the graph
	node = 0

	# checks if an edge already exists
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

	# add an ingoing edge, at the same time adds an outgoing edge in node
	def addInEdge(self, node):
		if(self.check(node, True)):
			self.ingoing.append(Edge(node, self, True))
			node.addOutEdge(self)

	# add an outgoing edge, at the same time adds an ingoing edge in node
	def addOutEdge(self, node):
		if(self.check(node, False)):
			self.outgoing.append(Edge(self, node, False))
			node.addInEdge(self)

	# remove an ingoing edge, at the same time removes an outgoing edge in node
	def removeInEdge(self, node):
		e = Edge(node, self, True)
		if e in self.ingoing:
			self.ingoing.remove(e)
			# node.removeOutEdge(self)

	# remove an outgoing edge, at the same time removes an ingoing edge in node
	def removeOutEdge(self, node):
		e = Edge(self, node, False)
		if e in self.outgoing:
			self.outgoing.remove(e)
			# node.removeInEdge(self)

	# get the degree of the node len(ingoing) + len(outgoing) edges
	def getDegree(self):
		return (len(self.ingoing) + len(self.outgoing))

	# gets a string representation of the coordinates
	def getCoords(self):
		return "x = " + str(self.goal.target_pose.pose.position.x) + \
					", y = " + str(self.goal.target_pose.pose.position.y) 
	# constructor
	def __init__(self, goal):
		self.goal = goal
		self.ingoing = []
		self.outgoing = []
		self.id = Node.node
		Node.node += 1

	# to string override
	def __str__(self):
		return str(self.id)

#main graph class
class Graph:

	# prints the graph: every node with all ingoing and outgoing edges
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

	# gets the node by its ID
	def getNodeByID(self, id):
		for node in self.nodes:
			if(node.id == id):
				return node
		return None

	# defines the edges between the nodes of the graph
	def defineEdges(self, edges):
		self.edges = list( set(self.edges) | set(edges))
		for edge in self.edges:
			fr = self.getNodeByID(edge[0])
			to = self.getNodeByID(edge[1])
			if(fr == None or to == None):
				raise Exception("Funny thing, no such ID")
			to.addInEdge(fr)
			fr.addOutEdge(to)

	def removeEdge(self, toRemove):
		try:
			(frID, toID) = self.edges[self.edges.index(toRemove)]
			self.edges.remove(toRemove)
			frNode = self.getNodeByID(frID)
			toNode = self.getNodeByID(toID)
			frNode.removeOutEdge(toNode)
			# toNode.removeOutEdge(frNode)
			return True
		except ValueError as e:
			print e
			return False

	# along with localize
	def calcDist(self, pose, node):
		x1 = pose.position.x
		y1 = pose.position.y
		x2 = node.goal.target_pose.pose.position.x
		y2 = node.goal.target_pose.pose.position.y
		# print x1, y1, "; ", x2, y2, "dist = ", math.sqrt( (x2 - x1)**2 + (y2 - y1)**2 )
		return math.sqrt( (x2 - x1) ** 2 + (y2 - y1) ** 2 )

	# method to determine the nearest node in the graph 
	# w.r.t. the current position of the roomba
	def localize(self, pose):
		minDist = sys.maxsize
		minNode = None
		for node in self.nodes:
			dist = self.calcDist(pose, node)
			if(dist < minDist):
				minDist = dist
				minNode = node
		return (minNode, minDist)

	# used by Astar
	def calcHeuristic(self, fr, to):
		x1 = fr.goal.target_pose.pose.position.x
		y1 = fr.goal.target_pose.pose.position.y
		x2 = to.goal.target_pose.pose.position.x
		y2 = to.goal.target_pose.pose.position.y
		# print x1, y1, "; ", x2, y2, "dist = ", math.sqrt( (x2 - x1)**2 + (y2 - y1)**2 )
		return math.sqrt( (x2 - x1)**2 + (y2 - y1)**2 )
	
	# used by Astar
	def minFScore(self, fScore, toVisit):
		minF = sys.maxsize
		node = None
		for n in fScore:
			if not(n in toVisit): 
				continue
			if fScore[n] < minF:
				node = n
				minF = fScore[n]
		return node

	# find the shortest path
	def Astar(self, fr, to):
		visited = set()
		toVisit = set()
		toVisit.add(fr.id)
		path = {}
		# set the scores
		gScore = {}
		fScore = {}
		for node in self.nodes:
			gScore[node.id] = sys.maxsize
			fScore[node.id] = sys.maxsize
		# set first values
		gScore[fr.id] = 0
		fScore[fr.id] = self.calcHeuristic(fr, to)
		# start iterating
		while len(toVisit):
			# get the minimal F-score
			current = self.minFScore(fScore, toVisit)
			currentNode = self.getNodeByID(current)
			# debugging info
			if currentNode == None:
				print "none!!!!", current
				oidhgkjsdfg
			# check if it the goal node, return the path start -> end
			if current == to.id:
				return self.reconstructPath(path, current)		
			# remove the current node from the queue of nodes to be visited
			toVisit.remove(current)
			# add to visited 
			visited.add(current)
			# iterate over outgoing edges of the current node
			for out in currentNode.outgoing: # out is instance of Edge class
				# get the node on the other side of the edge
				outNode = out.to
				outID = outNode.id
				# check if it was alredy visited
				if outID in visited:
					continue
				# get the score
				tentative_gScore = gScore[current] + self.calcHeuristic(currentNode, outNode)
				# check if it is in the queue
				if not(outID in toVisit):
					toVisit.add(outID)
				# if the score is too high skip it
				elif tentative_gScore >= gScore[current]:
					continue
				# set a path to this node, the heuristic and real cost
				path[outID] = current
				gScore[outID] = tentative_gScore
				fScore[outID] = gScore[outID] + self.calcHeuristic(outNode, to)
		# this means the algorithm did not find a path for some reason.
		return None

	# gets the path
	def reconstructPath(self, path, current):
		total_path = [current]
		while current in path:
			current = path[current]
			total_path = [current] + total_path
		return total_path

	# utils:
	def printSet(self, s):
		print "set elements = [",
		for e in s:
			print str(e) + ", ",
		print "]"

	def __init__(self, goals, edges):
		self.nodes = []
		for goal in goals:
			self.nodes.append(Node(goal))
		self.edges = []
		try:
			self.defineEdges(edges)
		except Exception as e:
			print e
