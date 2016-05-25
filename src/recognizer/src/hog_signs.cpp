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
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/ml/ml.hpp>

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <detection_msgs/Detection.h>

using namespace std;
using namespace cv;
using namespace ros;

#include "utilities.h"
#include "svmExtended.hpp"

Size reference_size(64, 128);
Subscriber sub;
vector<string> classes;
vector<Mat> HOGFeatures;
Mat trainSVM, labels;
CvSVM svm;

void load_signs(string dataset, vector<string> classes, vector<Mat>& signs, vector<int>& ids) {
	int class_id = 0;
	// ROS_INFO("Loading signs ...");
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

void buildHogDescriptors(vector<Mat> &signs, vector<int> &ids) {
	cout << "building desciptors ..." << endl;
	trainSVM = Mat(Size(3780, signs.size()), CV_32FC1);
	labels = Mat(Size(1, signs.size()), CV_32FC1);
	for(int i = 0; i < signs.size(); i++) {
		HOGDescriptor desc;
		vector<float> descriptors;
		desc.compute(signs.at(i), descriptors);
		labels.at<float>(0, i) = ids.at(i);
		// Mat featNow(descriptors.size(), 1, CV_32FC1);
		// cout << signs.at(i) << endl;
		// cout << i << " " << descriptors.size() << endl;
		for(int j = 0; j < descriptors.size(); j++) {
			trainSVM.at<float>(i, j) = descriptors.at(j);
		}
		// cout << featNow << endl;
		// break;
	}
	// cout << trainSVM.cols << " " << trainSVM.rows << endl;
	// imshow("img", labels);
	// waitKey(0);
}

void train() {
    cout << "training data ..." << endl;
    // Set up SVM's parameters
    CvSVMParams params;
    params.svm_type    = CvSVM::C_SVC;
    params.kernel_type = CvSVM::LINEAR;
    params.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);
	svm.train_auto(trainSVM, labels, Mat(), Mat(), params);
	// TODO: make a for loop to do one vs all!!
	// svm.save("svm.xml");
}

void test(string dataset) {
	cout << "testing ... " << endl;
	for (unsigned int i = 1; ; i++) {
		string filename = join(join(dataset, "testing"), format("%d.jpg", i));
		// cout << filename << endl;
		Mat test_image = imread(filename, IMREAD_GRAYSCALE);
		if (test_image.empty()) {
			break;
		}
		resize(test_image, test_image, reference_size);
		// svm.predict

		HOGDescriptor desc;
		vector<float> descriptors;
		desc.compute(test_image, descriptors);
		Mat t_img(Size(1, descriptors.size()), CV_32FC1);
		for(int j = 0; j < descriptors.size(); j++) {
			t_img.at<float>(i, j) = descriptors.at(j);
		}
		float ans = svm.predict(t_img, true);
		float confidence = 1.0 / (1.0 + exp(-ans));
		cout << ans << " " << confidence << endl;
	}
}

int main(int argc, char **argv) {
	// init(argc, argv, "sign_recognition");
	// NodeHandle node;
	string user_name(getenv("USER"));
	string dataset("/home/" + user_name + "/ROS/data/signs");
	string raw = "honk;left;limit;oneway;stop";
	classes = split(raw,';');
	std::vector<Mat> signs;
	vector<int> ids;
	load_signs(dataset, classes, signs, ids);
	buildHogDescriptors(signs, ids);
	train();
	test(dataset);

	// ROS_INFO("Training model ...");

	
	// ROS_INFO("Waiting for a sign to recognize ...");
	// sub = node.subscribe("/detector/traffic_signs", 100, recognize);
	// spin();
	return 0;
}
