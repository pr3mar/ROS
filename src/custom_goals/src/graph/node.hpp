#ifndef _NODE_HPP
#define _NODE_HPP
#pragma once

#include <iostream>
#include <cstdio>
#include <vector>
#include <string>
#include <fstream>
#include <stdlib.h>
#include <math.h>

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <detection_msgs/Detection.h> // actionlib at the moment, will reconsider 
// #include <geometry_msgs> // how to do this?
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalStatusArray.h>

#include "edge.hpp"

// the next line goes in the actual server!
//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Node
{
public:
	Node(move_base_msgs::MoveBaseGoal &goal);
	~Node();
	int getDegree();
	void addInEdge(Node in);
	void addOutEdge(Node out);
	move_base_msgs::MoveBaseGoal getGoal();
	int current_id;
private:
	static int node;
	move_base_msgs::MoveBaseGoal goal;
	Node *prev;
	std::vector<Edge> ingoing;
	std::vector<Edge> outgoing;
	float calc_dist();
};

#endif