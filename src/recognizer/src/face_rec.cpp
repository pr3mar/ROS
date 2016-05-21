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
//using namespace cv::face;

#include "utilities.h"

#define WINDOW_NAME "Face recognition"

Size reference_size(50, 50);

void load_faces(string dataset, vector<string> classes, vector<Mat>& faces, vector<int>& ids) {
  int class_id = 0;
  cout << "Loading faces ..." << endl;
  for (vector<string>::iterator it = classes.begin(); it != classes.end(); it++) {
    for (unsigned int i = 1; ; i++) {
        string filename = join(join(dataset, "training"), join(*it, format("%d.jpg", i)));
        cout << filename << endl;
        Mat image = imread(filename, IMREAD_GRAYSCALE);
        Mat resized;
        if (image.empty()) {
            cout << "image: " << filename << " is empty" << endl;
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

int main( int argc, char** argv ) {
  vector<Mat> faces;
  vector<int> ids;
  string user_name(getenv("USER"));
  string dataset("/home/" + user_name + "/ROS/data/faces"), raw;
  raw = "ellen;filip;forest;harry;kim;mathew;peter;scarlett;tina";
  vector<string> classes = split(raw, ';');
  load_faces(dataset, classes, faces, ids);
  Ptr<FaceRecognizer> recognizer = createEigenFaceRecognizer();
  cout << "Training model ..." << endl;
  recognizer->train(faces, ids);
  cout << "Testing model ..." << endl;
  for (unsigned int i = 1; ; i++) {
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
      /*cvtColor(image, visualization, COLOR_GRAY2BGR); 
      putText(visualization, format("%s (%.3f)", classes[label].c_str(), confidence), Point(10, 10), FONT_HERSHEY_SIMPLEX, 0.3,
        Scalar(255, 0, 0), 1, 8);
      imshow(WINDOW_NAME, visualization);
      waitKey(0);*/
      cout << i << " -> " << classes[label] << endl;
    }
  }
  return 0;
}



