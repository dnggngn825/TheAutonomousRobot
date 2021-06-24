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
// Including library
// ==================================================
#include "ros/ros.h"
#include <ros/package.h>
#include <bitset>
#include "amr/amr.h"
#include <math.h>
using namespace asclinic_pkg;


enum Control_Mode : int
{
	OPEN_LOOP 			= 0,
	PI_CONTROLLER_WO_AW = 1,
	PI_CONTROLLER_W_AW 	= 2,
};


// ==================================================
// Constants/Variables/Functions 
// ==================================================
// CONSTANTS
const float voltPrpm 	= AMR_MOTOR_VOLTAGE/POLOLU_SMC_G2_MOTOR_MAX_ANGULAR_VELOCITY_RPM_W_GEARBOX;
const float voltPrdps 	= voltPrpm/RDPS_P_RPM;

// Control variables
int motor_control_mode = Control_Mode::OPEN_LOOP;

// Global variables
ros::Publisher publisher_motor_pwm_duty_cycle;
float Kp_l = 1.9799f, Kd_l = 0.0f, Ki_l = 2.0802f;
float Kp_r = 1.5589f, Kd_r = 0.0f, Ki_r = 1.7385f; //Cz =  Kp + Ki*z/(z-1)

float angular_velocity_estimated_l = 0.0f, angular_velocity_estimated_r = 0.0f;
float angular_velocity_reference_l = 0.0f, angular_velocity_reference_r = 0.0f;

float voltage_l = 0.0f, voltage_r = 0.0f;

float abs_voltage_limit = AMR_MOTOR_VOLTAGE*0.4f;
int abs_pwm_limit = 20;


// No AW
float pre_error_dtheta_l = 0.0f, pre_error_dtheta_r = 0.0f;
float pre_ui_l = 0.0f, pre_ui_r = 0.0f;
// AW_EulerFW - AW_EulerBW
float pre_error_dtheta_awfb_l = 0.0f, pre_error_dtheta_awfb_r = 0.0f;
float pre_voltage_after_sat_l = 0.0f, pre_voltage_after_sat_r = 0.0f;


// Functions
void subscriberCallbackMotorAngularVelEst(const MotorAngularVelocity& msg);
void subscriberCallbackMotorAngularVelRef(const MotorAngularVelocity& msg);

float get_voltage_limit(float voltage);
int get_pwm_limit(int pwm_duty_cycle);

void getControlVoltage_OL();
void getControlVoltage_PID_NoAW();
// void getControlVoltage_PID_AW_EulerFW();
void getControlVoltage_PID_AW_EulerBW();

MotorAngularVelocityPWM getControlMotorAngularVelPWM();

float get_voltage_limit(float voltage);
int get_pwm_limit(int pwm_duty_cycle);

// ==================================================
// MAIN Program
// ==================================================
int main(int argc, char* argv[])
{
	ros::init(argc, argv, amr_node::MOTOR_CONTROLLER);
	ros::NodeHandle nd;
	
	ros::Subscriber subscriber_angular_vel_est = nd.subscribe(amr_topic::MOTOR_ANGULAR_VELOCITY_EST, 1, subscriberCallbackMotorAngularVelEst);
	ros::Subscriber subscriber_angular_vel_ref = nd.subscribe(amr_topic::MOTOR_ANGULAR_VELOCITY_REF, 1, subscriberCallbackMotorAngularVelRef);

	publisher_motor_pwm_duty_cycle = nd.advertise<MotorAngularVelocityPWM>(amr_topic::MOTOR_PWM_DUTY_CYCLE, 100, false);

	nd.getParam("motor_control_mode", motor_control_mode);

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
	angular_velocity_estimated_l = msg.angular_velocity_motor_l;
	angular_velocity_estimated_r = msg.angular_velocity_motor_r;

	// ROS_INFO_STREAM("[MOTOR_CONTROLLER] Estimated ang vel (L/R) "<< angular_velocity_estimated_l <<";" << angular_velocity_estimated_r);

	switch(motor_control_mode) 
	{
		case Control_Mode::PI_CONTROLLER_W_AW:
			getControlVoltage_PID_AW_EulerBW();
			break;
		case Control_Mode::PI_CONTROLLER_WO_AW:
			getControlVoltage_PID_NoAW();
			break;
		case Control_Mode::OPEN_LOOP:	
		default:
			getControlVoltage_OL();
	}
	
	MotorAngularVelocityPWM angular_vel_pwm = getControlMotorAngularVelPWM();

	publisher_motor_pwm_duty_cycle.publish(angular_vel_pwm);

	// // Display data
	// static int counter = 0;
	// counter++;
	// if (counter == 10)
	// {
	// 	ROS_INFO_STREAM("[MOTOR_CONTROLLER] Agular Velovity feedback (L/R): " << angular_velocity_estimated_l << ";" << angular_velocity_estimated_r);
	// 	counter = 0;
	// }
}

/**
 * @brief [Subscriber Callback] Subscribes to Motor Angular Velocity Reference.
 */
void subscriberCallbackMotorAngularVelRef(const MotorAngularVelocity& msg)
{
	angular_velocity_reference_l = msg.angular_velocity_motor_l;
	angular_velocity_reference_r = msg.angular_velocity_motor_r;
	// ROS_INFO_STREAM("[MOTOR_CONTROLLER] Ang Vel Ref (L/R): "<< angular_velocity_reference_l <<";"<<  angular_velocity_reference_r);

	pre_error_dtheta_l		= 0.0f; pre_error_dtheta_r 		= 0.0f;
	pre_ui_l 				= 0.0f; pre_ui_r 				= 0.0f;
	pre_error_dtheta_awfb_l = 0.0f; pre_error_dtheta_awfb_r = 0.0f;
	pre_voltage_after_sat_l = 0.0f; pre_voltage_after_sat_r = 0.0f;
}


// ==================================================
// Other Functions
// ==================================================
/**
 * @brief Generate the control signals using PID controller.
 * @return MotorAngularVelocity in PWM duty cycle
 */
void getControlVoltage_PID_NoAW()
{
	float error_dtheta_l = angular_velocity_reference_l - angular_velocity_estimated_l;
	float error_dtheta_r = angular_velocity_reference_r - angular_velocity_estimated_r;

	float up_l = Kp_l * error_dtheta_l;
	float up_r = Kp_r * error_dtheta_r;
	float ui_l = ui_l + Ki_l*error_dtheta_l;
	float ui_r = ui_r + Ki_r*error_dtheta_r;

	voltage_l = up_l + ui_l;
	voltage_r = up_r + ui_r;

	// ROS_INFO_STREAM("[MOTOR_CONTROLLER] Voltage (L/R): " <<voltage_l <<";"<<voltage_r);
	voltage_l = get_voltage_limit(voltage_l);
	voltage_r = get_voltage_limit(voltage_r);

	// Store pre values
	pre_ui_l = ui_l;
	pre_ui_r = ui_r;

	pre_error_dtheta_l = error_dtheta_l;
	pre_error_dtheta_r = error_dtheta_r;

}

/* 
void getControlVoltage_PID_AW_EulerFW()
{
	float error_dtheta_l = angular_velocity_reference_l - angular_velocity_estimated_l;
	float error_dtheta_r = angular_velocity_reference_r - angular_velocity_estimated_r;

	float error_dtheta_awfb_l = (pre_error_dtheta_awfb_l*(Kp_l-Ki_l) - Ki_l/Kp_l*pre_voltage_after_sat_l)/(Kp_l);
	float error_dtheta_awfb_r = (pre_error_dtheta_awfb_r*(Kp_l-Ki_l) - Ki_r/Kp_r*pre_voltage_after_sat_r)/(Kp_r);

	float error_dtheta_aw_l = error_dtheta_l - error_dtheta_awfb_l;
	float error_dtheta_aw_r = error_dtheta_r - error_dtheta_awfb_r;


	voltage_l = Kp_l * error_dtheta_l;
	voltage_r = Kp_r * error_dtheta_r;

	// ROS_INFO_STREAM("[MOTOR_CONTROLLER] Voltage (L/R): " <<voltage_l <<";"<<voltage_r);
	voltage_l = get_voltage_limit(voltage_l);
	voltage_r = get_voltage_limit(voltage_r);


	// Store pre values
	pre_voltage_after_sat_l = voltage_l;
	pre_voltage_after_sat_r = voltage_r;

	pre_error_dtheta_awfb_l = error_dtheta_awfb_l;
	pre_error_dtheta_awfb_r = error_dtheta_awfb_r;

	pre_error_dtheta_l = error_dtheta_l;
	pre_error_dtheta_r = error_dtheta_r;
}
 */

void getControlVoltage_PID_AW_EulerBW()
{
	float error_dtheta_l = angular_velocity_reference_l - angular_velocity_estimated_l;
	float error_dtheta_r = angular_velocity_reference_r - angular_velocity_estimated_r;

	float error_dtheta_awfb_l = 0.0f, error_dtheta_awfb_r = 0.0f;

	for (int i = 0; i<5; i++)
	{
		if (i==0)
		{
			error_dtheta_awfb_l = pre_error_dtheta_awfb_l;
			error_dtheta_awfb_r = pre_error_dtheta_awfb_r;
		}
		else
		{
			error_dtheta_awfb_l = (-Ki_l/Kp_l*voltage_l + Kp_l*pre_error_dtheta_awfb_l)/(Kp_l+Ki_l);
			error_dtheta_awfb_r = (-Ki_r/Kp_r*voltage_r + Kp_r*pre_error_dtheta_awfb_r)/(Kp_r+Ki_r);
		}
		
		voltage_l = Kp_l*(error_dtheta_l - error_dtheta_awfb_l);
		voltage_r = Kp_r*(error_dtheta_r - error_dtheta_awfb_r);
		
		voltage_l = get_voltage_limit(voltage_l);
		voltage_r = get_voltage_limit(voltage_r);
	}

	// ROS_INFO_STREAM("[MOTOR_CONTROLLER] Voltage (L/R): " <<voltage_l <<";"<<voltage_r);

	// Store pre values
	pre_error_dtheta_awfb_l = error_dtheta_awfb_l;
	pre_error_dtheta_awfb_r = error_dtheta_awfb_r;
}

void getControlVoltage_OL()
{
	voltage_l = (angular_velocity_reference_l)*voltPrdps;
	voltage_r = (angular_velocity_reference_r)*voltPrdps;

	// if (abs(voltage_l) < 0.6f || abs(voltage_r) < 0.6f)
	// {
	// 	voltage_l = 0;
	// 	voltage_r = 0;
	// }

	voltage_l = get_voltage_limit(voltage_l);
	voltage_r = get_voltage_limit(voltage_r);
}


MotorAngularVelocityPWM getControlMotorAngularVelPWM()
{
	float pwm_duty_cycle_l_f = 100.0f*voltage_l/AMR_MOTOR_VOLTAGE;
	float pwm_duty_cycle_r_f = 100.0f*voltage_r/AMR_MOTOR_VOLTAGE;

	int pwm_duty_cycle_l = round(pwm_duty_cycle_l_f);
	int pwm_duty_cycle_r = round(pwm_duty_cycle_r_f);

	MotorAngularVelocityPWM angular_vel_pwm = MotorAngularVelocityPWM();
	angular_vel_pwm.pwm_duty_cycle_motor_l = pwm_duty_cycle_l * AMR_DIRECTION_CW_MOTOR_L;
	angular_vel_pwm.pwm_duty_cycle_motor_r = pwm_duty_cycle_r * AMR_DIRECTION_CW_MOTOR_R;

	return angular_vel_pwm;
}


/**
 * @brief Get the voltage limit in range [-12; 12]
 * 
 * @param voltage desired voltage
 * @return [float] rescaled voltage
 */
float get_voltage_limit(float voltage)
{
	float returned_voltage;
	if (voltage > abs_voltage_limit)
		returned_voltage = abs_voltage_limit;
	else if (voltage < -abs_voltage_limit)
		returned_voltage = -abs_voltage_limit;
	else 
		returned_voltage = voltage;

	return returned_voltage;
}

/**
 * @brief Get the pwm limit in range [-100; 100]
 * 
 * @param pwm_duty_cycle desired pwm duty cycle
 * @return [int] rescaled pwm
 */
int get_pwm_limit(int pwm_duty_cycle)
{
	int returned_pwm_duty_cycle = 0;
	if (pwm_duty_cycle > abs_pwm_limit) 
		returned_pwm_duty_cycle = abs_pwm_limit;
	else if (pwm_duty_cycle < -abs_pwm_limit) 
		returned_pwm_duty_cycle = -abs_pwm_limit;
	// else if (abs(pwm_duty_cycle) < 4)
	// 	returned_pwm_duty_cycle = 0;
	// else if (pwm_duty_cycle < 5 && pwm_duty_cycle >=4.5)
	// 	returned_pwm_duty_cycle = 5;
	// else if (pwm_duty_cycle > -5 && pwm_duty_cycle <= -4.5)
	// 	returned_pwm_duty_cycle = -5;
	else 
		returned_pwm_duty_cycle;

	return returned_pwm_duty_cycle;
}



/* 
MotorAngularVelocityPWM getPIDControlMotorAngularVelPWM()
{
	float pwm_duty_cycle_l_f = 100.0f*voltage_l/AMR_MOTOR_VOLTAGE;
	float pwm_duty_cycle_r_f = 100.0f*voltage_r/AMR_MOTOR_VOLTAGE;

	int pwm_duty_cycle_l = round(pwm_duty_cycle_l_f);
	int pwm_duty_cycle_r = round(pwm_duty_cycle_r_f);

	// ROS_INFO_STREAM("[MOTOR_CONTROLLER] PWM (L/R): " <<pwm_duty_cycle_l <<";"<<pwm_duty_cycle_r);

	pwm_duty_cycle_l = get_pwm_limit(pwm_duty_cycle_l);
	pwm_duty_cycle_r = get_pwm_limit(pwm_duty_cycle_r);

	MotorAngularVelocityPWM angular_vel_pwm = MotorAngularVelocityPWM();
	angular_vel_pwm.pwm_duty_cycle_motor_l = pwm_duty_cycle_l * AMR_DIRECTION_CW_MOTOR_L;
	angular_vel_pwm.pwm_duty_cycle_motor_r = pwm_duty_cycle_r * AMR_DIRECTION_CW_MOTOR_R;

	return angular_vel_pwm;
}


MotorAngularVelocityPWM getDirectMotorAngularVelPWM()
{
	float pwm_duty_cycle_l_f = 100.0f*(angular_velocity_reference_l/(2*3.14)*60.0f)/(POLOLU_SMC_G2_MOTOR_MAX_ANGULAR_VELOCITY_RPM_W_GEARBOX);
	float pwm_duty_cycle_r_f = 100.0f*(angular_velocity_reference_r/(2*3.14)*60.0f)/(POLOLU_SMC_G2_MOTOR_MAX_ANGULAR_VELOCITY_RPM_W_GEARBOX);

	int pwm_duty_cycle_l = round(pwm_duty_cycle_l_f);
	int pwm_duty_cycle_r = round(pwm_duty_cycle_r_f);

	MotorAngularVelocityPWM angular_vel_pwm = MotorAngularVelocityPWM();
	angular_vel_pwm.pwm_duty_cycle_motor_l = pwm_duty_cycle_l * AMR_DIRECTION_CW_MOTOR_L;
	angular_vel_pwm.pwm_duty_cycle_motor_r = pwm_duty_cycle_r * AMR_DIRECTION_CW_MOTOR_R;

	return angular_vel_pwm;
}
 */
