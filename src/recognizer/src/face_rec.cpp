#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <detection_msgs/Detection.h>
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

using namespace std;
using namespace cv;
using namespace ros;
//using namespace cv::face;

#include "utilities.h"

#define WINDOW_NAME "Face recognition"

Size reference_size(50, 50);
Subscriber sub;
Ptr<FaceRecognizer> recognizer;
vector<string> classes;

void load_faces(string dataset, vector<string> classes, vector<Mat>& faces, vector<int>& ids) {
  int class_id = 0;
  ROS_INFO("Loading faces ...");
  for (vector<string>::iterator it = classes.begin(); it != classes.end(); it++) {
    for (unsigned int i = 1; ; i++) {
      string filename = join(join(dataset, "training"), join(*it, format("%d.jpg", i)));
      // cout << filename << endl;
      Mat image = imread(filename, IMREAD_GRAYSCALE);
      Mat resized;
      if (image.empty()) {
        // cout << "image: " << filename << " is empty" << endl;
        break;
      }
      resize(image, resized, reference_size);
      Mat sample = resized;
      if (!sample.empty()) {
        faces.push_back(sample);
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
  cout << resized.cols << " " << resized.rows << endl;
  Mat sample = resized;
  if (!sample.empty()) {
    int label;
    double confidence;
    recognizer->predict(sample, label, confidence);
    Mat visualization;
    // TODO: 
    cout << "I recognized: " << classes[label] << endl;
    // ros::ROS_INFO("I recognized: " + classes[label]);
  }
}

int main( int argc, char** argv ) {
  ros::init(argc, argv, "face_recognizer");
  NodeHandle node;
  vector<Mat> faces;
  vector<int> ids;
  string user_name(getenv("USER"));
  /* path to the database */
  string dataset("/home/" + user_name + "/ROS/data/faces"), raw;
  /**/
  raw = "ellen;filip;forest;harry;kim;mathew;peter;scarlett;tina";
  classes = split(raw, ';');
  load_faces(dataset, classes, faces, ids);
  
  recognizer = createEigenFaceRecognizer();
  ROS_INFO("Training model ...");
  recognizer->train(faces, ids);

  ROS_INFO("Waiting for face to recognize: ");
  /*for (unsigned int i = 1; ; i++) {
    string filename = join(join(dataset, "testing"), format("%d.jpg", i));
    Mat image = imread(filename, IMREAD_GRAYSCALE);
    if (image.empty())
      break;
    resize(image, image, reference_size);
    Mat sample = image;
    if (!sample.empty()) {
      int label;
      double confidence;
      recognizer->predict(sample, label, confidence);
      Mat visualization;
      // TODO: 
      cout << i << " -> " << classes[label] << endl;
    }
  }*/
  sub = node.subscribe("/facedetector/faces", 100, recognize);
  spin();
  return 0;
}


