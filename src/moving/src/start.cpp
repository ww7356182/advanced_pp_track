  /*#include "ros/ros.h"
  #include "std_msgs/String.h"
  #include <sstream>
  int main(int argc, char **argv)
  {
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000); 
    ros::Rate loop_rate(10); 
    while (ros::ok())
     {
         std_msgs::String msg; 
         std::stringstream ss;
         ss << "hello world " ;
       msg.data = ss.str();
     ROS_INFO("%s", msg.data.c_str());
     chatter_pub.publish(msg);
     ros::spinOnce();
    loop_rate.sleep();
   }
 return 0;
 }*/
  #include "ros/ros.h"
  #include "geometry_msgs/PoseStamped.h"
  #include<iostream>
  using namespace std;
  int main(int argc, char **argv)
  {
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("/initialpose", 1000); 
    ros::Rate loop_rate(0.5); 
   while (ros::ok())
     {
    geometry_msgs::PoseStamped msg;
    msg.header.stamp =ros::Time::now();
    msg.header.frame_id="map";
    msg.pose.position.x=10.24;
    msg.pose.position.y=15.55;
    msg.pose.position.z=0;
    msg.pose.orientation.x=0;
    msg.pose.orientation.y=0;
    msg.pose.orientation.z=0;
    msg.pose.orientation.w=1;
    chatter_pub.publish(msg);
    ros::spinOnce();
   loop_rate.sleep();
     }
    std::cout<<"output";


//break;
    //ros::spinOnce();
   //loop_rate.sleep();
 //}
 return 0;
 }
