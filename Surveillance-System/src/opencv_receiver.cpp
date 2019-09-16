#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>

void CheckStatus(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("[OpenCV_Receiver] The data receive is [%s] \n",msg->data.c_str());
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"openCV_receiver");
	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe("medium_man",1000,CheckStatus);
	ros::spin();
	
	return 0;
}
