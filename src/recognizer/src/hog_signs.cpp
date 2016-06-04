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
#include <std_msgs/String.h>
using namespace std;
using namespace cv;
using namespace ros;

#include "utilities.h"
#include "svmExtended.hpp"

Size reference_size(64, 128);
Subscriber sub;
Publisher pub;
vector<string> classes;
vector<Mat> HOGFeatures;
Mat trainSVM, labels;
vector<Ptr<CvSVM>> svms;
vector<Mat> allLabels;

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
		desc.compute(signs.at(i), descriptors, Size(32, 32), Size(0,0));
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
	imshow("img", trainSVM);
	waitKey(0);
}

void train(vector<int> &ids) {
    cout << "training data ..." << endl;
    // Set up SVM's parameters
    CvSVMParams params;
    params.svm_type    = CvSVM::C_SVC;//EPS_SVR;//C_SVC;//
    params.kernel_type = CvSVM::RBF;//POLY;//SIGMOID;//LINEAR;//
    // params.degree = 3;
    // params.p = 0.9;
    params.C = pow(2,-5);
    params.gamma = pow(2,-5);
    // params.coef0 = 0.001;
    // params.nu = 0.90;
    params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 10000, 1e-9);
    for(int i = 0; i < classes.size(); i++) {
    	cout << "training " << i << " ..." << endl;
    	Mat currentLabels(Size(1, trainSVM.rows), CV_32FC1);
    	for(int j = 0; j < ids.size(); j++) {
    		if(i == ids[j]) {
    			// cout << "positive ";
    			currentLabels.at<float>(j, 0) = 1;
    		}
    		else {
    			// cout << "negative ";
    			currentLabels.at<float>(j, 0) = -1;
    		}
    		// cout << currentLabels.at<float>(j,0) << endl;
    	}
    	// cout << currentLabels.rows << " " << currentLabels.cols << endl << endl;
    	// cout << currentLabels << endl << endl;
    	allLabels.push_back(currentLabels);
    	CvSVM *svm = new CvSVM;
    	svm->train_auto(trainSVM, currentLabels, Mat(), Mat(), params, 100);
    	string fname = "svm/" + to_string(i) + "_c_rbf.xml";
    	svm -> save(fname.c_str());
    	svms.push_back(svm);
    }
	
	// TODO: make a for loop to do one vs all!!
	// svm.save("svm.xml");
}

void load_svms(string path) {
	for(int i = 0; i < classes.size(); i++) {
		CvSVM *svm = new CvSVM;
		string p = join(path,format("%d_c_rbf.xml", i));
		svm -> load(p.c_str());
		svms.push_back(svm);
	}
}

void test(string dataset) {
	cout << "testing ... " << endl;
	for (unsigned int i = 1; ; i++) {
		string filename = join(join(dataset, "testing"), format("%d.jpg", i));
		cout << filename << endl;
		Mat test_image = imread(filename, IMREAD_GRAYSCALE);
		if (test_image.empty()) {
			break;
		}
		resize(test_image, test_image, reference_size);
		// svm.predict

		HOGDescriptor desc;
		vector<float> descriptors;
		desc.compute(test_image, descriptors, Size(32, 32), Size(0,0));
		Mat t_img(Size(1, descriptors.size()), CV_32FC1);
		for(int j = 0; j < descriptors.size(); j++) {
			t_img.at<float>(i, j) = descriptors.at(j);
		}
		float minVal = 1000000; int maxID = 0;
		for(int i = 0; i < svms.size(); i++) {
			float ans = svms[i] -> predict(t_img, true);
			float confidence = 1.0 / (1.0 + exp(-ans));
			if(minVal > confidence) {
				minVal = confidence;
				maxID = i;
			}
			// cout << classes[i] << " " << ans << endl;//" " << confidence << endl;
		}
		// cout << endl;
		cout << "final: " << minVal << " " << classes[maxID] << endl;
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
  // cout << resized.cols << " " << resized.rows << endl;
  Mat sample = resized;
  if (!sample.empty()) {
		// int label;
		// double confidence;
		// recognizer->predict(sample, label, confidence);
		
		HOGDescriptor desc;
		vector<float> descriptors;
		desc.compute(sample, descriptors, Size(32, 32), Size(0,0));
		Mat t_img(Size(1, descriptors.size()), CV_32FC1);
		for(int j = 0; j < descriptors.size(); j++) {
			t_img.at<float>(0, j) = descriptors.at(j);
		}
		float minVal = 1000000; int maxID = 0;
		for(int i = 0; i < svms.size(); i++) {
			float ans = svms[i] -> predict(t_img, true);
			float confidence = 1.0 / (1.0 + exp(-ans));
			if(minVal > confidence) {
				minVal = confidence;
				maxID = i;
			}
			// cout << classes[i] << " " << confidence << endl;
		}
		cout << "final: " << minVal << " " << classes[maxID] << endl << endl;
		std_msgs::String to_pub;
		std::stringstream ss;
    	ss << classes[maxID];
  	    to_pub.data = ss.str();
		pub.publish(to_pub);
  }
}
int main(int argc, char **argv) {
	init(argc, argv, "sign_recognition");
	NodeHandle node;
	string user_name(getenv("USER"));
	string dataset("/home/" + user_name + "/ROS/data/signs");
	string svm_path("/home/" + user_name + "/ROS/svm");
	string raw = "honk;left;limit;oneway;stop";
	classes = split(raw,';');
	std::vector<Mat> signs;
	vector<int> ids;
	// load_signs(dataset, classes, signs, ids);
	// buildHogDescriptors(signs, ids);
	// train(ids);
	load_svms(svm_path);
	// test(dataset);

	// ROS_INFO("Training model ...");

	
	ROS_INFO("Waiting for a sign to recognize ...");
	pub = node.advertise<std_msgs::String>("recognizer/signs", 1000);
	sub = node.subscribe("/detector/traffic_signs", 100, recognize);
	spin();
	return 0;
}

