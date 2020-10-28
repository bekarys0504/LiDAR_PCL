#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

bool cond = true;

void chatterCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZRGB> input;
  pcl::fromROSMsg (*msg, input);
  if (cond){
    pcl::io::savePCDFileASCII ("test_pcd.pcd", input);
    cond = false;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/camera/depth_registered/points", 1000, chatterCallback);
  ros::spin();

  return 0;
}