/* ==================================================
AUTHORSHIP STATEMENT
The University of Melbourne
School of Engineering
ELEN90090: Autonomous Clinic Systems
Author: Quang Trung Le (987445)
================================================== */


/* ==================================================
FUNCTIONALITY: This function uses PID controller to generate desired Voltages, sends them to motors.
INPUT: Desired angular velocity (using SYSTEM_TIMER) / feedback angular velocity (using MOTOR_TIMER).
OUTPUT: pwm_duty_cycle of 2 motors.
================================================== */



// ==================================================
// Constants/Variables/Functions 
// ==================================================
#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/UInt16.h"

#include <bitset>
#include "amr/amr.h"

using namespace asclinic_pkg;


// Global variables
ros::Publisher publisher_motor_pwm_duty_cycle;
float Kp_l = 0, Kd_l = 0, Ki_l = 0;
float Kp_r = 0, Kd_r = 0, Ki_r = 0;

float angular_velocity_motor_l, angular_velocity_motor_r;
float angular_velocity_reference_motor_l, angular_velocity_reference_motor_r;
float sum_error_dtheta_l = 0, pre_error_dtheta_l = 0;
float sum_error_dtheta_r = 0, pre_error_dtheta_r = 0;


// Functions
void subscriberCallbackMotorAngularVelEst(const MotorAngularVelocity& msg);
void subscriberCallbackMotorAngularVelRef(const MotorAngularVelocity& msg);
MotorAngularVelocityPWM getPIDControllerMotorAngularVel();


// ==================================================
// MAIN Program
// ==================================================
int main(int argc, char* argv[])
{
	ros::init(argc, argv, amr_node::MOTOR_CONTROLLER);
	ros::NodeHandle nodeHandle;
	
	ros::Subscriber subscriber_angular_vel_est = nodeHandle.subscribe(amr_topic::MOTOR_ANGULAR_VELOCITY_EST, 1, subscriberCallbackMotorAngularVelEst);
	ros::Subscriber subscriber_angular_vel_ref = nodeHandle.subscribe(amr_topic::MOTOR_ANGULAR_VELOCITY_REF, 1, subscriberCallbackMotorAngularVelRef);

	publisher_motor_pwm_duty_cycle = nodeHandle.advertise<MotorAngularVelocityPWM>(amr_topic::MOTOR_PWM_DUTY_CYCLE, 100, false);

	ros::spin();

	return 0;
}


// ==================================================
// CALLBACK Functions
// ==================================================
/**
 * @brief [Subscriber Callback] Subscribes to Motor Angular Velocity Estimation.
 */
void subscriberCallbackMotorAngularVelEst(const MotorAngularVelocity& msg)
{
	angular_velocity_motor_l = msg.angular_velocity_motor_l;
	angular_velocity_motor_r = msg.angular_velocity_motor_r;

	MotorAngularVelocityPWM angular_vel_pwm = getPIDControllerMotorAngularVel();

	publisher_motor_pwm_duty_cycle.publish(angular_vel_pwm);
}

/**
 * @brief [Subscriber Callback] Subscribes to Motor Angular Velocity Reference.
 */
void subscriberCallbackMotorAngularVelRef(const MotorAngularVelocity& msg)
{
	angular_velocity_reference_motor_l = msg.angular_velocity_motor_l;
	angular_velocity_reference_motor_r = msg.angular_velocity_motor_r;

	sum_error_dtheta_l = 0; sum_error_dtheta_r = 0;
	pre_error_dtheta_l = 0; pre_error_dtheta_r = 0;
}


// ==================================================
// Other Functions
// ==================================================
/**
 * @brief Generate the control signals using PID controller.
 * @return MotorAngularVelocity in PWM duty cycle
 */
MotorAngularVelocityPWM getPIDControllerMotorAngularVel()
{
	MotorAngularVelocityPWM angular_vel_pwm = MotorAngularVelocityPWM();

	float error_dtheta_l = angular_velocity_reference_motor_l - angular_velocity_motor_l;
	float error_dtheta_r = angular_velocity_reference_motor_r - angular_velocity_motor_r;

	sum_error_dtheta_l += error_dtheta_l;
	sum_error_dtheta_r += error_dtheta_r;

	float delta_error_dtheta_l = error_dtheta_l - pre_error_dtheta_l;
	float delta_error_dtheta_r = error_dtheta_r - pre_error_dtheta_r;

	float voltage_l = Kp_l*error_dtheta_l + Ki_l*sum_error_dtheta_l + Kd_l*delta_error_dtheta_l;
	float voltage_r = Kp_r*error_dtheta_r + Ki_r*sum_error_dtheta_r + Kd_r*delta_error_dtheta_r;

	int pwm_duty_cycle_l = round(voltage_l/AMR_MOTOR_VOLTAGE*100.0f);
	int pwm_duty_cycle_r = round(voltage_r/AMR_MOTOR_VOLTAGE*100.0f);

	angular_vel_pwm.pwm_duty_cycle_motor_l = pwm_duty_cycle_l;
	angular_vel_pwm.pwm_duty_cycle_motor_r = pwm_duty_cycle_r;

	return angular_vel_pwm;
}

