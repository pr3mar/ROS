#ifndef _EDGE_HPP
#define _EDGE_HPP
#pragma once

#include <iostream>
#include <vector>
#include <math.h>

class Node;

class Edge
{
public:
	Edge(Node &from_, Node &to_);
	~Edge() {};
private:
	float calc_dist();
	Node *from;
	Node *to;
	float cost;
};

#endif