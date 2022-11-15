#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <criocros/OCreply.h>

ros::Publisher pathPub;

void republishPath(const criocros::OCreply::ConstPtr& inMsg)
{
	nav_msgs::Path outMsg;
	outMsg.header = inMsg->header;
	outMsg.poses.resize(inMsg->Px.size());
	for (unsigned int i = 0; i != inMsg->Px.size(); i++)
	{
		outMsg.poses[i].header = inMsg->header;
		outMsg.poses[i].pose.position.x = inMsg->Px[i];
		outMsg.poses[i].pose.position.y = inMsg->Py[i];
		outMsg.poses[i].pose.position.z = 0.f;
		outMsg.poses[i].pose.orientation.x = 0.f;
		outMsg.poses[i].pose.orientation.y = 0.f;
		outMsg.poses[i].pose.orientation.z = 0.f;
		outMsg.poses[i].pose.orientation.w = 1.f;
	}
	pathPub.publish(outMsg);

	ROS_INFO("Hi");
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "display_crioc");
	ros::NodeHandle node;

	pathPub = node.advertise<nav_msgs::Path>("ocpath", 1);
	ros::Subscriber pathSub(node.subscribe("oc_reply", 1, republishPath));

	ros::spin();
}