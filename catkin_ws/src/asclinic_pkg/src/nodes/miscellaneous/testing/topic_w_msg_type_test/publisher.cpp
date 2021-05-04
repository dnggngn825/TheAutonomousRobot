#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/UInt32.h"
#include "asclinic_pkg/MotorAngularVelocityPWM.h"

using namespace asclinic_pkg;


// Declare "member" variables
ros::Publisher m_template_publisher;
ros::Timer m_timer_for_publishing;

// Respond to timer callback
void timerCallbackForPublishing(const ros::TimerEvent&)
{
	MotorAngularVelocityPWM msg = MotorAngularVelocityPWM();
	m_template_publisher.publish(msg);
}


int main(int argc, char* argv[])
{
	// Initialise the node
	ros::init(argc, argv, "publisher");
	ros::NodeHandle nodeHandle;
	// Initialise a publisher
	m_template_publisher = nodeHandle.advertise<MotorAngularVelocityPWM>("velocity", 10, false);
	// Initialise a timer
	// m_timer_for_publishing = nodeHandle.createTimer(ros::Duration(1.0), timerCallbackForPublishing, false);
	// Initialise a subscriber
	// ros::Subscriber template_subscriber = nodeHandle.subscribe("great_topic", 1, templateSubscriberCallback);
	// Spin as a single-threaded node
	ros::spin();

	return 0;
}
