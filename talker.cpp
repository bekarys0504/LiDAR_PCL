#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
using namespace std;

int main(int argc, char **argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr inlierPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("./src/test_pcl/src/test_pcd.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }

  //for(int nIndex = 0; nIndex < cloud->points.size(); nIndex++)
  //{
	//   cloud->points[nIndex].r = 255;
	//   cloud->points[nIndex].b = 0;
	//   cloud->points[nIndex].g = 0;
  // }

  // Filter object.
  //pcl::UniformSampling<pcl::PointXYZRGB> filter;
  pcl::VoxelGrid<pcl::PointXYZRGB> filter;
	filter.setInputCloud(cloud);	
	filter.setLeafSize(0.01f, 0.01f, 0.01f);
	filter.filter(*filteredCloud);
	
  pcl::PassThrough<pcl::PointXYZRGB> PTfilter;
	PTfilter.setInputCloud(filteredCloud);
	// Filter out all points with Z values not in the [0-2] range.
  PTfilter.setFilterFieldName("y");
	PTfilter.setFilterLimits(-0.13, 0.2);

	PTfilter.filter(*filteredCloud);

	PTfilter.setInputCloud(filteredCloud);
	// Filter out all points with Z values not in the [0-2] range.
  PTfilter.setFilterFieldName("z");
	PTfilter.setFilterLimits(0.7, 1.2); //sidushka stula

	PTfilter.filter(*filteredCloud);

	PTfilter.setInputCloud(filteredCloud);
	// Filter out all points with Z values not in the [0-2] range.
  PTfilter.setFilterFieldName("x");
	PTfilter.setFilterLimits(-0.5, 0.15); //z axis butilka

	PTfilter.filter(*filteredCloud);

  // Object for storing the plane model coefficients.
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	// Create the segmentation object.
	pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;
	segmentation.setInputCloud(filteredCloud);
	// Configure the object to look for a plane.
	segmentation.setModelType(pcl::SACMODEL_PLANE);
	// Use RANSAC method.
	segmentation.setMethodType(pcl::SAC_RANSAC);
	// Set the maximum allowed distance to the model.
	segmentation.setDistanceThreshold(0.01);
	// Enable model coefficient refinement (optional).
	segmentation.setOptimizeCoefficients(true);

  pcl::PointIndices inlierIndices;
	segmentation.segment(inlierIndices, *coefficients);

  if (inlierIndices.indices.size() == 0)
		std::cout << "Could not find any points that fitted the plane model." << std::endl;
	else
	{
		std::cerr << "Model coefficients: " << coefficients->values[0] << " "
				  << coefficients->values[1] << " "
				  << coefficients->values[2] << " "
				  << coefficients->values[3] << std::endl;

		// Copy all inliers of the model to another cloud.
		pcl::copyPointCloud<pcl::PointXYZRGB>(*filteredCloud, inlierIndices, *inlierPoints);
	}

  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  sensor_msgs::PointCloud2 cloud_msg;
  sensor_msgs::PointCloud2 full_input_msg;
  sensor_msgs::PointCloud2 ransac_plane_msg;
  
  // advertise
  ros::Publisher pub_full_input = n.advertise<sensor_msgs::PointCloud2>("full_input", 10);
  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("test_pcl", 10);
  ros::Publisher pub_ransac = n.advertise<sensor_msgs::PointCloud2>("ransac_plane", 10);
  // ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(60);
  
  int count = 0;
  
  
  std::ifstream inputFile("./src/test_pcl/src/test.txt");
  std::string line;
    
  while (ros::ok())
  {
    pcl::toROSMsg (*filteredCloud, cloud_msg);
    cloud_msg.header.frame_id = "camera";
    cloud_msg.header.stamp = ros::Time::now();

    pcl::toROSMsg (*cloud, full_input_msg);
    full_input_msg.header.frame_id = "camera";
    full_input_msg.header.stamp = ros::Time::now();

    pcl::toROSMsg (*inlierPoints, ransac_plane_msg);
    ransac_plane_msg.header.frame_id = "camera";
    ransac_plane_msg.header.stamp = ros::Time::now();

	  pub.publish(cloud_msg);
    pub_full_input.publish(full_input_msg);
    pub_ransac.publish(ransac_plane_msg);
    ros::spinOnce();
    loop_rate.sleep();
  
  }
  return 0;
}