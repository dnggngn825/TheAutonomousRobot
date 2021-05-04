/* ==================================================
AUTHORSHIP STATEMENT
The University of Melbourne
School of Engineering
ELEN90090: Autonomous Clinic Systems
Author: Quang Trung Le (987445)
================================================== */


/* ==================================================
FUNCTIONALITY: Reads PWM signal sent from: 1. "amr_controller" in minimal mode/ 2. "motor_controller" in full mode. Generate PWM duty cycle to motor. It acts like the physical component.
INPUT: None
OUTPUT: Timer of inner & outer loop

// Respond to subscriber receiving a message
// > To test this out without creating an additional
//   ROS node
//   1) First use the command:
//        rostopic list
//      To identify the full name of this subscription topic.
//   2) Then use the following command to send  message on
//      that topic
//        rostopic pub --once <namespace>/set_motor_duty_cycle std_msgs/UInt16 10
//      where "<namespace>/set_motor_duty_cycle" is the full
//      name identified in step 1.
//
// Eg: $ rostopic pub -once /motor_pwm_duty_cycle MotorAngularVelocityPWM '{pwm_duty_cycle_motor_l: 10, pwm_duty_cycle_motor_r:10}'
================================================== */



// ==================================================
// Including library
// ==================================================
#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/UInt16.h"
#include "amr/amr.h"
#include "amr/amr_constants.h"

#include <bitset>

// Namespace
using namespace asclinic_pkg;


// ==================================================
// Constants/Variables/Functions 
// ==================================================
const uint8_t MOTOR_FREQUENCY = 50;

// Specify the various limite
const int NEW_CURRENT_LIMIT_IN_MILLIAMPS = 5000;
const int NEW_MAX_SPEED_LIMIT = 2560;
const int NEW_MAX_ACCEL_LIMIT = 1;
const int NEW_MAX_DECEL_LIMIT = 5;


// Hardware setup for this node
// const char * m_i2c_device_name = "/dev/i2c-1";
// I2C_Driver m_i2c_driver (m_i2c_device_name);
I2C_Driver m_i2c_driver (AMR_I2C_DEVICE_NAME);

// const uint8_t m_pololu_smc_address_left = 13;
// const uint8_t m_pololu_smc_address_right = 14;
// Pololu_SMC_G2 m_pololu_smc_left (&m_i2c_driver, m_pololu_smc_address_left);
// Pololu_SMC_G2 m_pololu_smc_right (&m_i2c_driver, m_pololu_smc_address_right);
Pololu_SMC_G2 m_pololu_smc_left (&m_i2c_driver, AMR_POLOLU_SMC_ADDRESS_L);
Pololu_SMC_G2 m_pololu_smc_right (&m_i2c_driver, AMR_POLOLU_SMC_ADDRESS_R);

Pololu_SMC_G2 * pololu_smc_pointer_motor_l = &m_pololu_smc_left;
Pololu_SMC_G2 * pololu_smc_pointer_motor_r = &m_pololu_smc_right;


// Functions
int clip_pwm_duty_cycle(int pwm_duty_cycle);
void subscriberCallbackForMotorPwmDutyCycle(const MotorAngularVelocityPWM& msg);

void open_i2c_driver();
void close_i2c_driver();
void set_exit_safe_start(Pololu_SMC_G2 * pololu_smc_pointer);
void check_status_flag_registers(Pololu_SMC_G2 * pololu_smc_pointer);
void check_input_voltage(Pololu_SMC_G2 * pololu_smc_pointer);
void set_current_limit(Pololu_SMC_G2 * pololu_smc_pointer, bool check_result);
void set_max_speed_limit(Pololu_SMC_G2 * pololu_smc_pointer, bool check_result);
void set_max_accel_limit(Pololu_SMC_G2 * pololu_smc_pointer, bool check_result);
void set_max_decel_limit(Pololu_SMC_G2 * pololu_smc_pointer, bool check_result);


// ==================================================
// MAIN Program
// ==================================================
int main(int argc, char* argv[])
{
	ros::init(argc, argv, amr_node::MOTOR_DYNAMICS);
	ros::NodeHandle nodeHandle;
	// Initialise a publisher
	//ros::Publisher current_measurement_publisher = nodeHandle.advertise<std_msgs::UInt16>("current_measurement", 10, false);
	// Initialise a subscriber
	ros::Subscriber motor_pwm_duty_cycle_subscriber = nodeHandle.subscribe(amr_topic::MOTOR_PWM_DUTY_CYCLE, 1, subscriberCallbackForMotorPwmDutyCycle);

	open_i2c_driver();


	// SET THE CONFIGURATION OF EACH MOTOR DYNAMICS
	// Initialise one boolean variable for the result of all calls to Pololu_SMC_G2 functions
	Pololu_SMC_G2 * pololu_smc_pointer;
	// Iterate over the pololu objects
	for (int smc_pointer_index=0; smc_pointer_index<2; smc_pointer_index++)
	{
		// Point to the appropriate motor controller
		if (smc_pointer_index == 0)
			pololu_smc_pointer = pololu_smc_pointer_motor_l;
		else
			pololu_smc_pointer = pololu_smc_pointer_motor_r;

		// Send the "exit safe start" command
		set_exit_safe_start(pololu_smc_pointer);

		// > Check the status flag registers
		// check_status_flag_registers(pololu_smc_pointer);

		// > Check the input voltage
		// check_input_voltage(pololu_smc_pointer);
		// usleep(1000); // Short sleep

		// Set the current limit
		set_current_limit(pololu_smc_pointer, false);
		usleep(1000); // Short sleep

		// Send the max speed limit
		set_max_speed_limit(pololu_smc_pointer, false);
		usleep(1000); // Short sleep

		// Set the max acceleration limit
		set_max_accel_limit(pololu_smc_pointer, false);
		usleep(1000); // Short sleep
		
		// Set the max deceleration limit
		set_max_decel_limit(pololu_smc_pointer, false);
		usleep(1000); // Short sleep


		ROS_INFO_STREAM("[MOTOR DYNAMICS] Finished setting up the Pololu SMC with I2C address " << pololu_smc_pointer->get_i2c_address() );
	}


	// Enter a loop that continues while ROS is still running
	// while (ros::ok())
	// {
	// 	ros::spin();
	// } // END OF: "while (ros::ok())"
	ros::spin();
	// Close the I2C device
	close_i2c_driver();

	return 0;
}


// ==================================================
// CALLBACK Functions
// ==================================================
void subscriberCallbackForMotorPwmDutyCycle(const MotorAngularVelocityPWM& msg)
{
	// Clip the data to be in the range [0,100]
	int pwm_duty_cycle_motor_l = msg.pwm_duty_cycle_motor_l;
	int pwm_duty_cycle_motor_r = msg.pwm_duty_cycle_motor_r;

	ROS_INFO_STREAM("[MOTOR DYNAMICS] Message receieved with data = " << pwm_duty_cycle_motor_l << " ; " << pwm_duty_cycle_motor_r);

	pwm_duty_cycle_motor_l = clip_pwm_duty_cycle(pwm_duty_cycle_motor_l);
	pwm_duty_cycle_motor_r = clip_pwm_duty_cycle(pwm_duty_cycle_motor_r);

	// Set pwm_duty_cycle

	bool result_l = pololu_smc_pointer_motor_l->set_motor_target_speed_percent(pwm_duty_cycle_motor_l);
	bool result_r = pololu_smc_pointer_motor_r->set_motor_target_speed_percent(pwm_duty_cycle_motor_r);

	if (!result_l)
		ROS_INFO_STREAM("[MOTOR DYNAMICS] FAILED - Pololu SMC - set motor percent NOT successful for I2C address " << pololu_smc_pointer_motor_l->get_i2c_address() );
	
	if (!result_r)
		ROS_INFO_STREAM("[MOTOR DYNAMICS] FAILED - Pololu SMC - set motor percent NOT successful for I2C address " << pololu_smc_pointer_motor_r->get_i2c_address() );



	// Get the target speed value to check that it was set correctly
	int16_t current_target_speed_value_motor_l, current_target_speed_value_motor_r;
	result_l = pololu_smc_pointer_motor_l->get_target_speed_3200(&current_target_speed_value_motor_l);
	result_r = pololu_smc_pointer_motor_r->get_target_speed_3200(&current_target_speed_value_motor_r);

	if (result_l && result_r)
		ROS_INFO_STREAM("[MOTOR DYNAMICS] Pololu SMC - get target speed value returned: " << 
						current_target_speed_value_motor_l << ";" << current_target_speed_value_motor_r << ", for I2C address " << 
						pololu_smc_pointer_motor_l->get_i2c_address() << ";" << pololu_smc_pointer_motor_r->get_i2c_address());
	else if (!result_l)
		ROS_INFO_STREAM("[MOTOR DYNAMICS] FAILED - Pololu SMC - get target speed value NOT successful for I2C address " << pololu_smc_pointer_motor_l->get_i2c_address() );
	else if (!result_r)
		ROS_INFO_STREAM("[MOTOR DYNAMICS] FAILED - Pololu SMC - get target speed value NOT successful for I2C address " << pololu_smc_pointer_motor_r->get_i2c_address() );
}


// ==================================================
// Other Functions
// ==================================================
// Functionality: Clip pwm signal into range [-100;100]
int clip_pwm_duty_cycle(int pwm_duty_cycle)
{
	if (pwm_duty_cycle < -100)
		pwm_duty_cycle = -100;
	if (pwm_duty_cycle > 100)
		pwm_duty_cycle = 100;

	return pwm_duty_cycle;
}

// Functionality: Open the I2C device.
void open_i2c_driver()
{
	bool is_motor_open_success = m_i2c_driver.open_i2c_device();
	if (!is_motor_open_success)
		ROS_INFO_STREAM("[MOTOR DYNAMICS] FAILED to open I2C device named " << m_i2c_driver.get_device_name());
	else
		ROS_INFO_STREAM("[MOTOR DYNAMICS] Successfully opened named " << m_i2c_driver.get_device_name() << ", with file descriptor = " << m_i2c_driver.get_file_descriptor());
}

// Functionality: Close the I2C device.
void close_i2c_driver()
{
	bool close_success = m_i2c_driver.close_i2c_device();
	
	// Display the status
	if (!close_success)
		ROS_INFO_STREAM("[MOTOR DYNAMICS] FAILED to close I2C device named " << m_i2c_driver.get_device_name());
	else
		ROS_INFO_STREAM("[MOTOR DYNAMICS] Successfully closed device named " << m_i2c_driver.get_device_name());
}

// Functionality: Set the current limit
void set_current_limit(Pololu_SMC_G2 * pololu_smc_pointer, bool check_result)
{
	bool result;
	result = pololu_smc_pointer->set_current_limit_in_milliamps(NEW_CURRENT_LIMIT_IN_MILLIAMPS);
	if (!result)
		ROS_INFO_STREAM("[MOTOR DYNAMICS] FAILED - Pololu SMC - set current limit NOT successful for I2C address " << pololu_smc_pointer->get_i2c_address() );


	// Check the current limit that was set
	if (check_result)
	{
		uint16_t current_limit_value;
		result = pololu_smc_pointer->get_current_limit(&current_limit_value);
		if (result)
			ROS_INFO_STREAM("[MOTOR DYNAMICS] Pololu SMC - get current limit returned: " << current_limit_value << ", for I2C address " << pololu_smc_pointer->get_i2c_address() );
		else
			ROS_INFO_STREAM("[MOTOR DYNAMICS] FAILED - Pololu SMC - get current limit NOT successful for I2C address " << pololu_smc_pointer->get_i2c_address() );
	}

}

// Functionality: Set the max speed limit
void set_max_speed_limit(Pololu_SMC_G2 * pololu_smc_pointer, bool check_result)
{
	int max_speed_limit_response_code;
	bool result = pololu_smc_pointer->set_motor_limit_max_speed(NEW_MAX_SPEED_LIMIT, &max_speed_limit_response_code);
	if (!result)
		ROS_INFO_STREAM("[MOTOR DYNAMICS] FAILED - Pololu SMC - set max speed limit NOT successful with response code " << max_speed_limit_response_code << ", for I2C address " << pololu_smc_pointer->get_i2c_address() );

	if (check_result)
	{
		// Check the max speed limit that was set
		uint16_t max_speed_limit_value;
		result = pololu_smc_pointer->get_max_speed_forward(&max_speed_limit_value);
		if (result)
			ROS_INFO_STREAM("[MOTOR DYNAMICS] Pololu SMC - get max speed limit returned: " << max_speed_limit_value << ", for I2C address " << pololu_smc_pointer->get_i2c_address() );
		else
			ROS_INFO_STREAM("[MOTOR DYNAMICS] FAILED - Pololu SMC - get max speed limit NOT successful for I2C address " << pololu_smc_pointer->get_i2c_address() );
	}
}

// Functionality: Set the max acceleration limit
void set_max_accel_limit(Pololu_SMC_G2 * pololu_smc_pointer, bool check_result)
{
	int max_accel_limit_response_code;
	bool result = pololu_smc_pointer->set_motor_limit_max_acceleration(NEW_MAX_ACCEL_LIMIT, &max_accel_limit_response_code);
	if (!result)
		ROS_INFO_STREAM("[MOTOR DYNAMICS] FAILED - Pololu SMC - set max acceleration limit NOT successful with response code " << max_accel_limit_response_code << " for I2C address " << pololu_smc_pointer->get_i2c_address() );

	if (check_result)
	{
		// Check the max speed acceleration that was set
		uint16_t max_accel_limit_value;
		result = pololu_smc_pointer->get_max_acceleration_forward(&max_accel_limit_value);
		if (result)
			ROS_INFO_STREAM("[MOTOR DYNAMICS] Pololu SMC - get max acceleration limit returned: " << max_accel_limit_value << ", for I2C address " << pololu_smc_pointer->get_i2c_address() );
		else
			ROS_INFO_STREAM("[MOTOR DYNAMICS] FAILED - Pololu SMC - get max acceleration limit NOT successful for I2C address " << pololu_smc_pointer->get_i2c_address() );
	}
}

// Functionality: Set the max deceleration limit
void set_max_decel_limit(Pololu_SMC_G2 * pololu_smc_pointer, bool check_result)
{
	int max_decel_limit_response_code;
	bool result = pololu_smc_pointer->set_motor_limit_max_deceleration(NEW_MAX_DECEL_LIMIT, &max_decel_limit_response_code);
	if (!result)
		ROS_INFO_STREAM("[MOTOR DYNAMICS] FAILED - Pololu SMC - set max deceleration limit NOT successful with response code " << max_decel_limit_response_code << " for I2C address " << pololu_smc_pointer->get_i2c_address() );

	if (check_result)
	{
		// > Check the max speed deceleration that was set
		uint16_t max_decel_limit_value;
		result = pololu_smc_pointer->get_max_deceleration_forward(&max_decel_limit_value);
		if (result)
			ROS_INFO_STREAM("[MOTOR DYNAMICS] Pololu SMC - get max deceleration limit returned: " << max_decel_limit_value << ", for I2C address " << pololu_smc_pointer->get_i2c_address() );
		else
			ROS_INFO_STREAM("[MOTOR DYNAMICS] FAILED - Pololu SMC - get max deceleration limit NOT successful for I2C address " << pololu_smc_pointer->get_i2c_address() );
	}
}

// Functionality: Check the input voltage
void check_input_voltage(Pololu_SMC_G2 * pololu_smc_pointer)
{
	float input_voltage_value;
	bool result = pololu_smc_pointer->get_input_voltage_in_volts(&input_voltage_value);
	if (result)
		ROS_INFO_STREAM("[MOTOR DYNAMICS] Pololu SMC - get input voltage value returned: " << input_voltage_value << " [Volts], for I2C address " << pololu_smc_pointer->get_i2c_address() );
	else
		ROS_INFO_STREAM("[MOTOR DYNAMICS] FAILED - Pololu SMC - get input voltage value NOT successful for I2C address " << pololu_smc_pointer->get_i2c_address() );

}

// Functionality: Check the status flag registers
void check_status_flag_registers(Pololu_SMC_G2 * pololu_smc_pointer)
{
	uint16_t error_status;
	bool result = pololu_smc_pointer->get_error_status(&error_status);
	if (result)
		ROS_INFO_STREAM("[MOTOR DYNAMICS] Pololu SMC - get error status returned: " << std::bitset<16>(error_status) << ", for I2C address " << pololu_smc_pointer->get_i2c_address() );
	else
		ROS_INFO_STREAM("[MOTOR DYNAMICS] FAILED - Pololu SMC - get error status NOT successful for I2C address " << pololu_smc_pointer->get_i2c_address() );
}

// Functionality: Send the "exit safe start" command
void set_exit_safe_start(Pololu_SMC_G2 * pololu_smc_pointer)
{
	bool result = pololu_smc_pointer->exit_safe_start();
	if (!result)
		ROS_INFO_STREAM("[MOTOR DYNAMICS] FAILED - Pololu SMC - exit safe start NOT successful for I2C address " << pololu_smc_pointer->get_i2c_address() );
}
