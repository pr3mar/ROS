#include "node.hpp"

using namespace std;

Edge::Edge(Node &from_, Node &to_) {
	from = &from_;
	to = &to_;
	cost = calc_dist();
}

float Edge::calc_dist() {
	float x1 = to -> getGoal() -> target_pose.pose.position.x;
	float y1 = to -> getGoal() -> target_pose.pose.position.y;

	float x2 = from -> getGoal() -> target_pose.pose.position.x;
	float y2 = from -> getGoal() -> target_pose.pose.position.y;

	return sqrt((pow(x1,2) - pow(x2,2)) + (pow(y1,2) - pow(y2,2)));
	// return -1;
}