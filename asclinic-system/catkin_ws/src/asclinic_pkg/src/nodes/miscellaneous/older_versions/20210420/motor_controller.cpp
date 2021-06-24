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

#include "i2c_driver/i2c_driver.h"
#include "pololu_smc_g2/pololu_smc_g2.h"

#include <bitset>

#include "amr/amr.h"
#include "pololu_smc_g2_encoder/pololu_smc_g2_encoder.h"

#include "asclinic_pkg/MotorAngularVelocity.h"
#include "asclinic_pkg/AmrPose.h"
#include "asclinic_pkg/MotorRotationAngle.h"

using namespace asclinic_pkg;


// Global variables
ros::Publisher motor_pwm_duty_cycle_publisher;
float Kp = 0, Kd = 0, Ki = 0;

float angular_velocity_motor_l, angular_velocity_motor_r;
float angular_velocity_reference_motor_l, angular_velocity_reference_motor_r;


// Functions
void subscriberCallbackMotorRotationAngle(const MotorRotationAngle& msg);
void subscriberCallbackMotorAngularVelReference(const MotorAngularVelocity& msg);
MotorAngularVelocity getPIDControllerMotorAngularVel();


void subscriberCallbackMotorRotationAngle(const MotorRotationAngle& msg)
{
	float rotation_angle_motor_l = msg.rotation_angle_motor_l;
	float rotation_angle_motor_r = msg.rotation_angle_motor_r;

	angular_velocity_motor_l = rotation_angle_motor_l/AMR_TIMER_4_MOTOR_CONTROL;
	angular_velocity_motor_r = rotation_angle_motor_r/AMR_TIMER_4_MOTOR_CONTROL;

	
	MotorAngularVelocity angular_vel = getPIDControllerMotorAngularVel();

	motor_pwm_duty_cycle_publisher.publish(angular_vel);
}

void subscriberCallbackMotorAngularVelReference(const MotorAngularVelocity& msg)
{
	angular_velocity_reference_motor_l = msg.angular_velocity_motor_l;
	angular_velocity_reference_motor_r = msg.angular_velocity_motor_r;
}

MotorAngularVelocity getPIDControllerMotorAngularVel()
{
	MotorAngularVelocity angular_vel = MotorAngularVelocity();
	return angular_vel;
}


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "motor_controller");
	ros::NodeHandle nodeHandle;
	
	ros::Subscriber motor_rotation_angle_subcriber = nodeHandle.subscribe("motor_rotation_angle", 1, subscriberCallbackMotorRotationAngle);
	ros::Subscriber motor_angular_vel_reference_subscriber = nodeHandle.subscribe("motor_angular_vel_reference", 1, subscriberCallbackMotorAngularVelReference);


	motor_pwm_duty_cycle_publisher = nodeHandle.advertise<MotorAngularVelocity>("motor_pwm_duty_cycle", 100, false);

	ros::spin();

	return 0;
}
