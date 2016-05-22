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
			// cout << filename << endl;
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

void recognize(const detection_msgs::DetectionConstPtr &det) {
	cv_bridge::CvImageConstPtr cv_ptr;
	cv_ptr = cv_bridge::toCvShare(det -> image, det, sensor_msgs::image_encodings::BGR8);
	if (cv_ptr -> image.empty())
		return;
	Mat resized;
	cvtColor(cv_ptr -> image, resized, CV_BGR2GRAY, 1);
	resize(resized, resized, reference_size);
	Mat sample = resized;
	if (!sample.empty()) {
		int label;
		double confidence;
		recognizer->predict(sample, label, confidence);
		Mat visualization;
    // TODO: 
		cout << "I recognized: " << classes[label] <<  " with confidence " << confidence << endl;
    // ros::ROS_INFO("I recognized: " + classes[label]);
	}
}


int main(int argc, char **argv) {
	init(argc, argv, "sign_recognition");
	NodeHandle node;
	string user_name(getenv("USER"));
	string dataset("/home/" + user_name + "/ROS/data/signs");
	string raw = "honk;left;limit;oneway;stop";
	classes = split(raw,';');
	std::vector<Mat> signs;
	vector<int> ids;
	load_signs(dataset, classes, signs, ids);

	ROS_INFO("Training model ...");
	recognizer = createEigenFaceRecognizer();
	recognizer->train(signs, ids);
	
	ROS_INFO("Waiting for a sign to recognize ...");
	sub = node.subscribe("/detector/traffic_signs", 100, recognize);
	spin();
	return 0;
}