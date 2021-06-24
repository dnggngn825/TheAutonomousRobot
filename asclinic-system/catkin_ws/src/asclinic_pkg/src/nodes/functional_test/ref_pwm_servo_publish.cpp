/* ==================================================
AUTHORSHIP STATEMENT
The University of Melbourne
School of Engineering
ELEN90090: Autonomous Clinic Systems
Author: Quang Trung Le (987445)
================================================== */


#include "ros/ros.h"
#include <ros/package.h>
#include <string>
#include <bitset>
#include "amr/amr.h"

using namespace asclinic_pkg;


// Global variables
ros::Publisher publisher_pwm;
ros::Timer m_timer_for_publishing;
int TIMER_DURATION = 5;

int pwm[] = {500,1500,2500,1500};

// Functions
void timerCallbackForPublishing(const ros::TimerEvent&);


int main(int argc, char* argv[])
{
	ros::init(argc, argv, amr_node::REF_PWM_PUBLISH);
	ros::NodeHandle nd;
	
	publisher_pwm 	= nd.advertise<ServoPulseWidth>(amr_topic::SERVO_CONTROL_SIGNAL, 100, false);

    m_timer_for_publishing = nd.createTimer(ros::Duration(TIMER_DURATION), timerCallbackForPublishing, true);
	
	ros::spin();
	
	return 0;
}


void timerCallbackForPublishing(const ros::TimerEvent&)
{
	static uint trajectory_counter = 0;
	static bool is_motor_stop = false;

    ServoPulseWidth msg = ServoPulseWidth();
	if(!is_motor_stop)
	{
		if (trajectory_counter < sizeof(pwm)/sizeof(pwm[0]))
		{
			ROS_INFO("PWM duty cycle sending.");
			msg.channel = 15;
			msg.pulse_width_in_microseconds = pwm[trajectory_counter];

			publisher_pwm.publish(msg);

			trajectory_counter++;
		}
		else
		{
			ROS_INFO("Stop sending PWM duty cycle.");
			is_motor_stop = true;
		}
	}

	// START THE TIMER AGAIN
	m_timer_for_publishing.stop();
	m_timer_for_publishing.setPeriod( ros::Duration(TIMER_DURATION), true);
	m_timer_for_publishing.start();
}


