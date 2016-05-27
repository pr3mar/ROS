#include <iostream>
#include <cstdio>
#include <vector>
#include <string>
#include <fstream>
#include <stdlib.h>

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <detection_msgs/Detection.h>
// #include <geometry_msgs> // how to do this?
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib_msgs/GoalStatusArray.h>


class Graph {
public:
	Graph();
private:

};