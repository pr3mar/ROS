/* HOG DETECTOR
 *
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <dlib/svm_threaded.h>
#include <dlib/image_processing.h>
#include <dlib/data_io.h>
#include <dlib/image_transforms.h>
#include <dlib/opencv.h>

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <vector>

#include <detection_msgs/Detection.h>

//added stuff
#include <sensor_msgs/CameraInfo.h>
#include <time.h>
#include <cmath>        // std::abs
#include <tf/transform_listener.h>

//#include "multiclass.h"

using namespace std;
using namespace dlib;

typedef scan_fhog_pyramid<pyramid_down<6> > image_scanner_type;
object_detector<image_scanner_type> object_detector_function;
ros::Publisher pub;
unsigned long message_counter = 0;


vector<sensor_msgs::CameraInfo> camera_infos;
void camera_callback(sensor_msgs::CameraInfo data) {
    camera_infos.push_back(data);
    if(camera_infos.size() > 50)
        camera_infos.erace(camera_infos.begin());
}

void detectCallback(const sensor_msgs::ImageConstPtr& cam_msg){
	cv_bridge::CvImagePtr cv_ptr;
	const unsigned long upsample_amount = 0;
	try {
		cv_ptr = cv_bridge::toCvCopy(cam_msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// dlib wrapper for OpenCV
	cv_image<bgr_pixel> image = cv_image<bgr_pixel>(cv_ptr->image);
	std::vector<std::pair<double, rectangle> > rects;
    object_detector_function(image, rects);

    cv::Rect image_region(0, 0, cv_ptr->image.cols, cv_ptr->image.rows);

	for (int i = 0; i < rects.size(); i++) {
		std::pair<double, rectangle> cur = rects[i];
        cv_image<bgr_pixel> chip;
        cv::Rect region(cur.second.left(), cur.second.top(), cur.second.width(), cur.second.height());
        cv_bridge::CvImage cvi(cv_ptr->header, cv_ptr->encoding, cv_ptr->image(region & image_region));

		detection_msgs::Detection msg = detection_msgs::Detection();
        msg.header.seq = message_counter++;
        msg.header.stamp = cam_msg->header.stamp;
        msg.header.frame_id = cam_msg->header.frame_id;
		msg.x = cur.second.left();
		msg.y = cur.second.top();
		msg.height = cur.second.height();
		msg.width = cur.second.width();
        msg.confidence = cur.first;
        msg.source = "dlib";

        //adding location
        std::double u = msg.x + msg.width / 2;
        std::double v = msg.y + msg.height / 2;
        
        sensor_msgs::CameraInfo camera_info;
        std::time_t best_time = 1;
        for(j = 0; j < camera_infos.size(); j++) {
            ROS_INFO("camera info loop");
            std::time_t time = abs(j.header.stamp.toSec() - msg.header.stamp.toSec());
            if (time < best_time) {
                ROS_INFO("Setting camera_info!");
                camera_info = j;
                best_time = time;
                break;
            }    
        }        
                
        if (!camera_info) {
            msg.label = "napaka!";
        }
        else {
        	image_geometry::PinholeCameraModel camera_model = image_geometry::PinholeCameraModel();
            camera_model.fromCameraInfo(camera_info);
            
            geometry_msgs::Point point = geometry_msgs::Point(((u - camera_model.cx()) - camera_model.Tx()) / camera_model.fx(), ((v - camera_model.cy()) - camera_model.Ty()) / camera_model.fy(), 1);
            
            localization = localizer::Localize(msg.header, point, 3);
            //localizer::Localize localization = localizer::Localize(msg.header, point, self.region_scope);
            
            if (!localization) {
                msg.label = "napaka localization!";
            }
            else {
                //we transform the point
                geometry_msgs::PoseStamped tmpPose = geometry_msgs::PoseStamped();
                tmpPose.header = msg.header;
                tmpPose.pose = localization.pose;
                try {
                    self.listener.waitForTransform(msg.header.frame_id, '/map', msg.header.stamp, ros::Duration(4.5))
                    ret = self.listener.transformPose('/map', tmpPose)
                }
                catch(int e) {
                    continue;
                }
                
                localization.pose = ret.pose;
              
                msg.label = to_string(localization.pose.position.x)+";"+to_string(localization.pose.position.y)+";"+to_string(localization.pose.position.z);
            }    

        }


        cvi.toImageMsg(msg.image);
		pub.publish(msg);
	}

}

int main(int argc, char** argv) {

    string detector_file, classifier_file;


    //added
    //self.region_scope = rospy.get_param('~region', 3)
    ros::service::waitForService("localizer");
    tf::TransformListener listener = tf::TransformListener();
    localizer::Localize localization = ros::ServiceProxy('localizer/localize', Localize)

	ros::init (argc, argv, "object_detector");
	ros::NodeHandle nh;
	ros::NodeHandle nhp("~");

	ros::Subscriber sub = nh.subscribe ("camera", 1, detectCallback);
	ros::Subscriber sub = nh.subscribe ("/camera/rgb/camera_info", 1, camera_callback);

	pub = nh.advertise<detection_msgs::Detection> ("detections", 1);

    nhp.param<string>("detector", detector_file, string(""));

    if (detector_file.empty()) 
        return -1;

	dlib::deserialize (detector_file) >> object_detector_function;

	ros::spin();
	return 0;
}
