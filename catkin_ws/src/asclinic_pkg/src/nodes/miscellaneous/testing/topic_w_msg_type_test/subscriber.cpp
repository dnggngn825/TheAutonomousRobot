#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/UInt32.h"
#include "asclinic_pkg/MotorAngularVelocityPWM.h"

using namespace asclinic_pkg;


// Declare "member" variables
// ros::Publisher m_template_publisher;

// Respond to subscriber receiving a message
void templateSubscriberCallback(const MotorAngularVelocityPWM& msg)
{
	ROS_INFO_STREAM("[TEMPLATE CPP NODE MINIMAL] Message receieved");
}

int main(int argc, char* argv[])
{
	// Initialise the node
	ros::init(argc, argv, "subscriber");
	ros::NodeHandle nodeHandle;

	// Initialise a subscriber
	ros::Subscriber template_subscriber = nodeHandle.subscribe("velocity", 1, templateSubscriberCallback);
	// Spin as a single-threaded node
	ros::spin();

	return 0;
}
