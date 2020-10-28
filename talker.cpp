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
using namespace std;

int main(int argc, char **argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  

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
  pcl::VoxelGrid<pcl::PointXYZRGB> filter;
	filter.setInputCloud(cloud);	
  // We set the size of every voxel to be 1x1x1cm
	// (only one point per every cubic centimeter will survive).
	filter.setLeafSize(0.01f, 0.01f, 0.01f);

	filter.filter(*filteredCloud);


  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  sensor_msgs::PointCloud2 cloud_msg;
  
  // advertise
  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("test_pcl", 10);
  // ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(60);
  
  int count = 0;
  
  
  std::ifstream inputFile("./src/test_pcl/src/test.txt");
  std::string line;
    
  while (ros::ok())
  {
    
    
    //std::istringstream ss(line);
    //std::string heart;

    //ss >> heart;
	  //pcl_conversions::moveFromPCL(cloud, cloud_msg);
    pcl::toROSMsg (*filteredCloud, cloud_msg);
    cloud_msg.header.frame_id = "camera";
    cloud_msg.header.stamp = ros::Time::now();
 
    //msg.data = ss.str();
    //chatter_pub.publish(msg);
	  pub.publish(cloud_msg);
    ros::spinOnce();
    loop_rate.sleep();

    ros::Publisher my_pub;
  
  }
  return 0;
}