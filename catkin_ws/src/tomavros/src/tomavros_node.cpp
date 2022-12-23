#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

ros::Publisher posepub;
bool first_Flag = false;
double first_msg[3];

void poseCallback(const nav_msgs::OdometryConstPtr& msg)
{
		geometry_msgs::PoseStamped pubmsg;
		pubmsg.header = msg->header;
		pubmsg.pose = msg->pose.pose;
		if(!first_Flag)
		{
				first_Flag = true;
				first_msg[0] = msg->pose.pose.position.x;
				first_msg[1] = msg->pose.pose.position.y;
				first_msg[2] = msg->pose.pose.position.z;
		}
		else
		{
				pubmsg.pose.position.x = pubmsg.pose.position.x - first_msg[0];
				pubmsg.pose.position.y = pubmsg.pose.position.y - first_msg[1];
				pubmsg.pose.position.z = pubmsg.pose.position.z - first_msg[2];
		}
		
		posepub.publish(pubmsg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "toamvros");
    ros::NodeHandle n("~");
    
    ros::Subscriber posesub = n.subscribe("/t265/odom/sample", 1000, poseCallback);
    posepub = n.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1000);
    
    ros::spin();
		
    return 0;
}
