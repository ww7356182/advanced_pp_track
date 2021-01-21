#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include<vector>
#include<algorithm>


void scanCallback(const sensor_msgs::LaserScanConstPtr &scan_msg)
{
    int size=scan_msg->ranges.size();
    std::vector<double> r;
    for(int i=0;i<size;i++)
    {
        r.push_back(scan_msg->ranges[i]);
    }
    std::sort(&r[0],&r[size-1]);
    printf("%f\n",r[0]);
}

  int main (int argc, char **argv)
{
  ros::init(argc, argv, "dec");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("scan", 1000, scanCallback);
  ros::spin();
  return (0);
}

