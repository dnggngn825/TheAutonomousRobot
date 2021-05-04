/* ==================================================
AUTHORSHIP STATEMENT
The University of Melbourne
School of Engineering
ELEN90090: Autonomous Clinic Systems
Author: Quang Trung Le (987445)
================================================== */


#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/UInt16.h"
#include <string>
#include <bitset>
#include "amr/amr.h"

using namespace asclinic_pkg;


int states_list[1000][2];
int states_list_length = 0;

// Global variables
ros::Publisher publisher_motor_pwm_duty_cycle;
ros::Publisher publisher_motor_stop;
ros::Timer m_timer_for_publishing;
int TIMER_DURATION = 10;


int pwm_duty_cycle_motor_l[] = {10,-10,10,-10,0};
int pwm_duty_cycle_motor_r[] = {-10,10,-10,10,0};

// Functions
void timerCallbackForPublishing(const ros::TimerEvent&);
void write_data();


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "motor_controller_sample");
	ros::NodeHandle nd;
	
	publisher_motor_pwm_duty_cycle = nd.advertise<MotorAngularVelocityPWM>(amr_topic::MOTOR_PWM_DUTY_CYCLE, 100, false);
	publisher_motor_stop = nd.advertise<std_msgs::UInt16>("motor_stop", 100, false);


    m_timer_for_publishing = nd.createTimer(ros::Duration(TIMER_DURATION), timerCallbackForPublishing, true);
	
	ros::spin();
	
	return 0;
}


/* void timerCallbackForPublishing(const ros::TimerEvent&)
{
	static uint trajectory_counter = 0;

	static bool is_motor_stop = false;

    MotorAngularVelocityPWM motor_angular_vel_pwm = MotorAngularVelocityPWM();
	if(!is_motor_stop)
	{
		if (trajectory_counter < sizeof(pwm_duty_cycle_motor_l)/sizeof(pwm_duty_cycle_motor_l[0]))
		{
			ROS_INFO("count");
			motor_angular_vel_pwm.pwm_duty_cycle_motor_l = pwm_duty_cycle_motor_l[trajectory_counter];
			motor_angular_vel_pwm.pwm_duty_cycle_motor_r = pwm_duty_cycle_motor_r[trajectory_counter];
			states_list[trajectory_counter][0] = pwm_duty_cycle_motor_l[trajectory_counter];
			states_list[trajectory_counter][1] = pwm_duty_cycle_motor_r[trajectory_counter];
			states_list_length++;

			publisher_motor_pwm_duty_cycle.publish(motor_angular_vel_pwm);
		}
		else
		{
			ROS_INFO("stop");
			motor_angular_vel_pwm.pwm_duty_cycle_motor_l = 0;
			motor_angular_vel_pwm.pwm_duty_cycle_motor_r = 0;
			publisher_motor_pwm_duty_cycle.publish(motor_angular_vel_pwm);
			publisher_motor_stop.publish(motor_angular_vel_pwm);
			write_data();
			is_motor_stop = true;
		}
	}

	// START THE TIMER AGAIN
	m_timer_for_publishing.stop();
	m_timer_for_publishing.setPeriod( ros::Duration(TIMER_DURATION), true);
	m_timer_for_publishing.start();

	trajectory_counter++;
} */


void timerCallbackForPublishing(const ros::TimerEvent&)
{
	static uint trajectory_counter = 0;
	static bool is_motor_stop = false;

    MotorAngularVelocityPWM motor_angular_vel_pwm = MotorAngularVelocityPWM();
	if(!is_motor_stop)
	{
		if (trajectory_counter < sizeof(pwm_duty_cycle_motor_l)/sizeof(pwm_duty_cycle_motor_l[0]))
		{
			ROS_INFO("PWM duty cycle sending.");
			motor_angular_vel_pwm.pwm_duty_cycle_motor_l = pwm_duty_cycle_motor_l[trajectory_counter];
			motor_angular_vel_pwm.pwm_duty_cycle_motor_r = pwm_duty_cycle_motor_r[trajectory_counter];
			// states_list[trajectory_counter][0] = pwm_duty_cycle_motor_l[trajectory_counter];
			// states_list[trajectory_counter][1] = pwm_duty_cycle_motor_r[trajectory_counter];
			// states_list_length++;

			publisher_motor_pwm_duty_cycle.publish(motor_angular_vel_pwm);

			trajectory_counter++;
		}
		else
		{
			ROS_INFO("Stop sending PWM duty cycle.");
			std_msgs::UInt16 msg;
			publisher_motor_stop.publish(msg);
			is_motor_stop = true;
		}
	}

	// START THE TIMER AGAIN
	m_timer_for_publishing.stop();
	m_timer_for_publishing.setPeriod( ros::Duration(TIMER_DURATION), true);
	m_timer_for_publishing.start();
}






