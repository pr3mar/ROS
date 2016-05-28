#ifndef _EDGE_HPP
#define _EDGE_HPP
#pragma once

#include <iostream>
#include <vector>
#include <math.h>

#include "node.hpp"

class Node;

class Edge
{
public:
	Edge(Node *from_, Node *to_);
	~Edge();
private:
	Node *from;
	Node *to;
	float cost;
};

#endif