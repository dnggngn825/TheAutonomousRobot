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


int states_list[1000][2];

// Global variables
ros::Publisher publisher_4_ref_vel;
// ros::Publisher publisher_4_stop;

ros::Timer m_timer_for_publishing;
int TIMER_DURATION = 5;

float vel_rdps_5pc = 1.05;
// float ref_vel_rdps_l[] = {vel_rdps_5pc, vel_rdps_5pc*2, vel_rdps_5pc*3, vel_rdps_5pc*4, 0};
// float ref_vel_rdps_r[] = {-vel_rdps_5pc, -vel_rdps_5pc*2, -vel_rdps_5pc*3, -vel_rdps_5pc*4, 0};

float ref_vel_rdps_l[] = {2.0, 4.0, 0};
float ref_vel_rdps_r[] = {2.0, 4.0, 0};

// Functions
void timerCallbackForPublishing(const ros::TimerEvent&);


int main(int argc, char* argv[])
{
	ros::init(argc, argv, amr_node::REF_ANG_VEL_PUBLISH);
	ros::NodeHandle nd;
	
	publisher_4_ref_vel 	= nd.advertise<MotorAngularVelocity>(amr_topic::MOTOR_ANGULAR_VELOCITY_REF, 100, false);
	// publisher_4_stop        = nd.advertise<amr_msgs::StopIndicator>(amr_topic::STOP, 100, false);

    m_timer_for_publishing 	= nd.createTimer(ros::Duration(TIMER_DURATION), timerCallbackForPublishing, true);
	
	ros::spin();
	
	return 0;
}


void timerCallbackForPublishing(const ros::TimerEvent&)
{
	static int trajectory_counter = 0;
	static bool is_motor_stop = false;

    MotorAngularVelocity ref_angular_vel = MotorAngularVelocity();
	if(!is_motor_stop)
	{
		if (trajectory_counter < sizeof(ref_vel_rdps_l)/sizeof(ref_vel_rdps_l[0]))
		{
			float ref_angular_vel_l = ref_vel_rdps_l[trajectory_counter];
			float ref_angular_vel_r = ref_vel_rdps_r[trajectory_counter];

			ref_angular_vel.angular_velocity_motor_l = ref_angular_vel_l;
			ref_angular_vel.angular_velocity_motor_r = ref_angular_vel_r;

			publisher_4_ref_vel.publish(ref_angular_vel);

			trajectory_counter++;
		}
		else
		{
			ROS_INFO("[REF_ANG_VEL_PUBLISH] Stop sending reference angular velocity.");
			// amr_msgs::EncCounter msg;
			// publisher_4_stop.publish(msg);
			is_motor_stop = true;
		}
	}

	// START THE TIMER AGAIN
	m_timer_for_publishing.stop();
	m_timer_for_publishing.setPeriod( ros::Duration(TIMER_DURATION), true);
	m_timer_for_publishing.start();
}


