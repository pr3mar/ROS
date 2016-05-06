#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

ros::Publisher pub;

// This function is a callback for incoming pointcloud data
void callback (const pcl::PCLPointCloud2ConstPtr& cloud_blob ) {

   pcl::PCLPointCloud2::Ptr cloud_filtered_blob(new pcl::PCLPointCloud2);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
          cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>),
          cloud_p(new pcl::PointCloud<pcl::PointXYZRGB>),
          cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);

   //Create the filtering object:downsample the dataset using a leaf size of 1cm
   pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
   sor.setInputCloud(cloud_blob);
   sor.setLeafSize(0.01f, 0.01f, 0.01f);
   sor.filter(*cloud_filtered_blob);

   // Convert to the templated PointCloud
   pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);
   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
   pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
   
   // Segment the point cloud
   pcl::SACSegmentation<pcl::PointXYZRGB> seg;
   seg.setOptimizeCoefficients(true);
   seg.setModelType(pcl::SACMODEL_PLANE);
   seg.setMethodType(pcl::SAC_RANSAC);
   seg.setMaxIterations(1000);
   seg.setDistanceThreshold(0.001);
   seg.setInputCloud(cloud_filtered);
   seg.segment(*inliers, *coefficients);

   // Exit if no plane found
   if(inliers -> indices.size() == 0) return;
 
   // Extract points of found plane
   pcl::ExtractIndices<pcl::PointXYZRGB> extract;
   extract.setInputCloud(cloud_filtered);
   extract.setIndices(inliers);
   extract.setNegative(false);
   extract.filter(*cloud_f);

   // Publish the plane to a new topic .
   pcl::PCLPointCloud2 outcloud;
   pcl::toPCLPointCloud2(*cloud_f, outcloud);
   pub.publish(outcloud);
}

/*
void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);

  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud.makeShared ());
  seg.segment (inliers, coefficients);

  // Publish the model coefficients
  pcl_msgs::ModelCoefficients ros_coefficients;
  pcl_conversions::fromPCL(coefficients, ros_coefficients);
  pub.publish (ros_coefficients);
}*/

int main (int argc, char** argv)
{
    // Initialize ROS
   ros::init (argc, argv, "my_pcl_tutorial");
   ros::NodeHandle nh;

   // Create a ROS subscriber for the input point cloud
   ros::Subscriber sub = nh.subscribe ("input", 1, callback);

   // Create a ROS publisher for the output point cloud
   pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
   //pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);
   
   // Spin
   ros::spin ();
}
