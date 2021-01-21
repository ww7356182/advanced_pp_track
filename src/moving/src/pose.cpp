#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "math.h"
#include "tf/transform_datatypes.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include<canlib.h>
#include<stdio.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <endian.h>
#include<Eigen/Core>
#include<Eigen/Geometry>
#include <Eigen/Dense>
int main(int argc, char** argv){
  ros::init(argc, argv, "tf_listener");
 
  ros::NodeHandle node;
 double x,y,theta,yaw,pitch,roll;
 tf::Quaternion q;
 int i=0;
 
  tf::TransformListener listener;
 
  ros::Rate rate(2);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/map", "/laser",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
 
    geometry_msgs::Twist vel_msg;
    //turtle_vel.publish(vel_msg);
    x=transform.getOrigin().x();
    y=transform.getOrigin().y(); 
    q=transform.getRotation(); 
    tf::Matrix3x3(q).getRPY(roll,pitch,yaw);

    printf("tar_x[%d]=%f;\ntar_y[%d]=%f;\ntar_theta[%d]=%f;\n",i,x,i,y,i,yaw*57.29578);
    i++;
    rate.sleep();
  }
  return 0;
}