/*
	Author : Keerthi Kumar
	Description: The received pcl from the Kinect/Simulator is segmented to generate a new pcl having only planar surfaces.
	Note: Based on pcl tutorials.
*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>


#define MIN_HEIGHT  0.5
#define MAX_HEIGHT 1.2

ros::Publisher pub; // ROS publisher to publish the segmented pcl.

// Process the received pcl and segment planar points.
void process_cloud (const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); // Received pcl
  pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>); // Segmented pcl
  sensor_msgs::PointCloud2::Ptr final_cloud (new sensor_msgs::PointCloud2), cloud_filtered_blob (new sensor_msgs::PointCloud2); // Segmented pcl in ROS type.

  // Convert from ROS type.
  pcl::fromROSMsg (*input, *cloud);
  std::vector<int> inliers;

  // Create RandomSampleConsensus object and compute the appropriated model.
  pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZ> (cloud));

  model_p->setAxis (Eigen::Vector3f (0.0, 1.0, 0.0));
  model_p->setEpsAngle (pcl::deg2rad (15.0));

  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
  ransac.setDistanceThreshold (.05);
  ransac.computeModel();
  ransac.getInliers(inliers);

  std::vector<int>::iterator it;
  std::vector<int> filtered_inliers;

  // for (it = inliers.begin(); it != inliers.end(); it++)

  //   {
  //     if ( MIN_HEIGHT <= cloud->points[*it].y &&  cloud->points[*it].y <= MAX_HEIGHT) 
  //       {
  //         filtered_inliers.push_back(*it);
  //       }
  //     else
  //       std::cout << "filterd point:" << cloud->points[*it].y << std::endl;
  //   }


  // Copy all inliers of the model computed to another PointCloud.
  pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);

  // Convert to ROS sensormsg type.
  pcl::toROSMsg (*final, *final_cloud);

  //std::cerr << "pcl cloud: " << cloud->width << " points" << std::endl;
  //std::cerr << "pcl final: " << final->width << " points" << std::endl;
  //std::cerr << "pcl final_cloud: " << final_cloud->width << " points" << std::endl;

  // Publish the segmented pcl.
  final_cloud->header.frame_id = input->header.frame_id;
  final_cloud->header.stamp = input->header.stamp;
  pub.publish (final_cloud);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_tabletop");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud.
  ros::Subscriber sub = nh.subscribe ("/head_xtion/depth/points", 1, process_cloud); // "/head_xtion/depth/points" is the pcl2 received from kinect/morse simulator.
  
  // Create a ROS publisher for the segmented point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("segmented_pcl", 1000); // Segmented pcl having only planar points.

  // Spin.
  ros::spin ();
}
