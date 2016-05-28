#include "node.hpp"

using namespace std;
using namespace ros;


int Node::node = 0;

Node::Node(move_base_msgs::MoveBaseGoal &goal) {
	this -> goal = goal;
	prev = null;
	id = node;
	node++;
}

void Node::addInEdge(Node in) {
	this -> ingoing.push_back(Edge(this, in));
}

void Node::addOutEdge(Node out) {
	this -> outgoing.push_back(Edge(out, this));
}

int Node::getDegree() {
	return this -> ingoing.size() + this -> outgoing.size();
}

move_base_msgs::MoveBaseGoal getGoal() {
	return goal;
}

bool Node::operator==(const Node &other) const {
	return (id == other.id);
}

bool Node::operator!=(const Node &other) const {
	return !(*this == other);
}