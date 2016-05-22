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
#include <opencv2/ml/ml.hpp>

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <detection_msgs/Detection.h>

using namespace std;
using namespace cv;
using namespace ros;

#include "utilities.h"

Size reference_size(20, 20);
int k = reference_size.width * reference_size.height;
Subscriber sub;
Ptr<FaceRecognizer> recognizer;
vector<string> classes;
Mat trainData, res_trainData;

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
			resized = resized.reshape(0, 1);
			Mat sample = resized;
			if (!sample.empty()) {
				signs.push_back(sample);
				ids.push_back(class_id);
			}
		}
		class_id++;
	}
	trainData = Mat::zeros(signs.size(), k, CV_32FC1);
	res_trainData = Mat::zeros(signs.size(), 1, CV_32FC1);
	cout << "cols: " << trainData.cols << ", rows:" << trainData.rows << endl;
	for(int i = 0; i < signs.size(); i++) {
		// cout << "cols: " <<signs[i].cols << ", rows:" << signs[i].rows << endl;
		signs[i].row(0).copyTo(trainData.row(i));
	}
	// imshow("This is madafuking gud", trainData);
	// waitKey(0);
}

void recognize() {

}

int main(int argc, char **argv) {
	init(argc, argv, "sign_knn_recognition");
	NodeHandle node;
	string user_name(getenv("USER"));
	string dataset("/home/" + user_name + "/ROS/data/signs");
	string raw = "honk;left;limit;oneway;stop";
	classes = split(raw,';');
	std::vector<Mat> signs;
	vector<int> ids;
	load_signs(dataset, classes, signs, ids);
	CvKNearest knn(trainData, res_trainData, Mat(), false, k);
	// TODO: test if this is what you expected it is.

	return 0;
}
