#include <ros/ros.h>
#include <cstdio>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <stdlib.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <detection_msgs/Detection.h>

using namespace std;
using namespace cv;
using namespace ros;

#include "utilities.h"

Size reference_size(50, 50);
Subscriber sub;
Ptr<FaceRecognizer> recognizer;
vector<string> classes;

void load_signs(string dataset, vector<string> classes, vector<Mat>& signs, vector<int>& ids) {
	int class_id = 0;
	ROS_INFO("Loading signs ...");
	for (vector<string>::iterator it = classes.begin(); it != classes.end(); it++) {
		for (unsigned int i = 1; ; i++) {
			string filename = join(join(dataset, "training"), join(*it, format("%d.jpg", i)));
			Mat image = imread(filename, IMREAD_GRAYSCALE);
			Mat resized;
			if (image.empty()) {
				break;
			}
			resize(image, resized, reference_size);
			Mat sample = resized;
			if (!sample.empty()) {
				signs.push_back(sample);
				ids.push_back(class_id);
			}
		}
		class_id++;
	}
}


int main(int argc, char **argv) {
	init(argc, argv, "sign_recognition");
	string user_name(getenv("USER"));
	string dataset("/home/" + user_name + "/ROS/data/signs");
	string raw = "honk;left;limit;oneway;stop";
	std::vector<Mat> signs;
	vector<int> ids;
	load_signs(dataset, classes, signs, ids);
	

	spin();
	return 0;
}