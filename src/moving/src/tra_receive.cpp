#include "ros/ros.h"
//#include "beginner_tutorials/RFID.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>


double i,j,k,f,x,y;
int count;

void chatterCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
//i++;
//i=msg->markers[3].pose.position.y;
//y=msg->markers.data;
ROS_INFO("start\n");
for(count=0;count<1455;count=count+5)
{
x=msg->markers[1].points[count].x;
y=msg->markers[1].points[count].y;
ROS_INFO("x:%f,y:%f\n",x,y);
}

i=msg->markers[0].points.size();
j=msg->markers[1].points.size();
k=msg->markers[2].points.size();
//y=msg->markers[0].pose.position.x;
ROS_INFO("i:%f,j:%f,k:%f,x:%f,y:%f",i,j,k,x,y);
ROS_INFO("end\n");
//msg->size;

}

int main(int argc, char **argv)
{
ros::init(argc, argv, "listener");
ros::NodeHandle n;
ros::Subscriber sub = n.subscribe("trajectory_node_list", 100, chatterCallback);
ros::spin();
return 0;
}
