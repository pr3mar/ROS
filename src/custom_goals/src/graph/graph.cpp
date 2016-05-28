#include "graph.hpp"
#include "node.hpp"
#include "edge.hpp"

using namespace ros;
using namespace std;


Graph::Graph(vector<move_base_msgs::MoveBaseGoal> goals) { // constructor
	for(int i = 0; i < goals.size(); i++) {
		Node n(goals[i]);
		nodes.push_back(n);
	}
}

void Graph::dijkstra(Node *from, Node *to) {

}

int main() {

	return 0;
}
