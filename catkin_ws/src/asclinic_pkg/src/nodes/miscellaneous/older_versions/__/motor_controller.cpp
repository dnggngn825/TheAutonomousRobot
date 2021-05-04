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

#include "asclinic_pkg/MotorAngularVelocityPWM.h"
// #include "asclinic_pkg/PoseData.h"


const uint8_t MOTOR_FREQUENCY = 50;

// Specify the various limite
const int NEW_CURRENT_LIMIT_IN_MILLIAMPS = 5000;
const int NEW_MAX_SPEED_LIMIT = 2560;
const int NEW_MAX_ACCEL_LIMIT = 1;
const int NEW_MAX_DECEL_LIMIT = 5;


// Hardware setup for this node
const char * m_i2c_device_name = "/dev/i2c-1";
I2C_Driver m_i2c_driver (m_i2c_device_name);

const uint8_t m_pololu_smc_address_left = 13;
const uint8_t m_pololu_smc_address_right = 14;
Pololu_SMC_G2 m_pololu_smc_left (&m_i2c_driver, m_pololu_smc_address_left);
Pololu_SMC_G2 m_pololu_smc_right (&m_i2c_driver, m_pololu_smc_address_right);

Pololu_SMC_G2 * pololu_smc_pointer_motor_l = &m_pololu_smc_left;
Pololu_SMC_G2 * pololu_smc_pointer_motor_r = &m_pololu_smc_right;


// Global variables
// float PoseData pose_ref = PoseData();



// Functions
int clip_pwm_duty_cycle(int pwm_duty_cycle);
void setMotorDutyCycleSubscriberCallback(const MotorAngularVelocityPWM& msg)

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
void setMotorDutyCycleSubscriberCallback(const MotorAngularVelocityPWM& msg)
{
	// ROS_INFO_STREAM("[MOTOR CONTROLLER] Message receieved with data = " << msg.data);

	// Clip the data to be in the range [0,100]
	int pwm_duty_cycle_motor_l = msg.pwm_duty_cycle_motor_l;
	int pwm_duty_cycle_motor_r = msg.pwm_duty_cycle_motor_r;

	pwm_duty_cycle_motor_l = clip_pwm_duty_cycle(pwm_duty_cycle_motor_l);
	pwm_duty_cycle_motor_r = clip_pwm_duty_cycle(pwm_duty_cycle_motor_r);

	// Set pwm_duty_cycle
	bool result_l = pololu_smc_pointer_motor_l->set_motor_target_speed_percent(pwm_duty_cycle_motor_l);
	bool result_r = pololu_smc_pointer_motor_r->set_motor_target_speed_percent(pwm_duty_cycle_motor_r);

	if (!result_l)
		ROS_INFO_STREAM("[MOTOR CONTROLLER] FAILED - Pololu SMC - set motor percent NOT successful for I2C address " << pololu_smc_pointer_motor_l->get_i2c_address() );
	
	if (!result_r)
		ROS_INFO_STREAM("[MOTOR CONTROLLER] FAILED - Pololu SMC - set motor percent NOT successful for I2C address " << pololu_smc_pointer_motor_r->get_i2c_address() );



	// Get the target speed value to check that it was set correctly
	int16_t current_target_speed_value_motor_l, current_target_speed_value_motor_r;
	bool result_l = pololu_smc_pointer_motor_l->get_target_speed_3200(&current_target_speed_value_motor_l);
	bool result_r = pololu_smc_pointer_motor_r->get_target_speed_3200(&current_target_speed_value_motor_r);

	if (result_l && result_r)
		ROS_INFO_STREAM("[MOTOR CONTROLLER] Pololu SMC - get target speed value returned: " << 
						current_target_speed_value_motor_l << ";" << current_target_speed_value_motor_r << ", for I2C address " << 
						pololu_smc_pointer_motor_l->get_i2c_address() << ";" << pololu_smc_pointer_motor_r->get_i2c_address());
	else if (!result_l)
		ROS_INFO_STREAM("[MOTOR CONTROLLER] FAILED - Pololu SMC - get target speed value NOT successful for I2C address " << pololu_smc_pointer_motor_l->get_i2c_address() );
	else if (!result_r)
		ROS_INFO_STREAM("[MOTOR CONTROLLER] FAILED - Pololu SMC - get target speed value NOT successful for I2C address " << pololu_smc_pointer_motor_r->get_i2c_address() );
}


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "motor_controller");
	ros::NodeHandle nodeHandle("~");
	// Initialise a publisher
	//ros::Publisher current_measurement_publisher = nodeHandle.advertise<std_msgs::UInt16>("current_measurement", 10, false);
	// Initialise a subscriber
	ros::Subscriber set_motor_duty_cycle_subscriber = nodeHandle.subscribe("set_motor_duty_cycle", 1, setMotorDutyCycleSubscriberCallback);

	ros::Rate loop_rate(MOTOR_FREQUENCY);

	// Open the I2C device
	// > Note that the I2C driver is already instantiated as a member variable of this node
	bool is_motor_open_success = m_i2c_driver.open_i2c_device();

	if (!is_motor_open_success)
		ROS_INFO_STREAM("[MOTOR CONTROLLER] FAILED to open I2C device named " << m_i2c_driver.get_device_name());
	else
		ROS_INFO_STREAM("[MOTOR CONTROLLER] Successfully opened named " << m_i2c_driver.get_device_name() << ", with file descriptor = " << m_i2c_driver.get_file_descriptor());


	// SET THE CONFIGURATION OF EACH MOTOR CONTROLLER
	// Initialise one boolean variable for the result of all calls to Pololu_SMC_G2 functions
	bool result;
	Pololu_SMC_G2 * pololu_smc_pointer;
	// Iterate over the pololu objects
	for (int i_smc=0; i_smc<2; i_smc++)
	{
		// Point to the appropriate motor controller
		if (i_smc == 0)
			pololu_smc_pointer = pololu_smc_pointer_motor_l;
		else
			pololu_smc_pointer = pololu_smc_pointer_motor_r;

		// Send the "exit safe start" command
		result = pololu_smc_pointer->exit_safe_start();
		if (!result)
			ROS_INFO_STREAM("[MOTOR CONTROLLER] FAILED - Pololu SMC - exit safe start NOT successful for I2C address " << pololu_smc_pointer->get_i2c_address() );

		// > Check the status flag registers
		uint16_t error_status;
		result = pololu_smc_pointer->get_error_status(&error_status);
		if (result)
			ROS_INFO_STREAM("[MOTOR CONTROLLER] Pololu SMC - get error status returned: " << std::bitset<16>(error_status) << ", for I2C address " << pololu_smc_pointer->get_i2c_address() );
		else
			ROS_INFO_STREAM("[MOTOR CONTROLLER] FAILED - Pololu SMC - get error status NOT successful for I2C address " << pololu_smc_pointer->get_i2c_address() );


		// > Check the input voltage
		float input_voltage_value;
		result = pololu_smc_pointer->get_input_voltage_in_volts(&input_voltage_value);
		if (result)
			ROS_INFO_STREAM("[MOTOR CONTROLLER] Pololu SMC - get input voltage value returned: " << input_voltage_value << " [Volts], for I2C address " << pololu_smc_pointer->get_i2c_address() );
		else
			ROS_INFO_STREAM("[MOTOR CONTROLLER] FAILED - Pololu SMC - get input voltage value NOT successful for I2C address " << pololu_smc_pointer->get_i2c_address() );

		// Short sleep
		usleep(1000);


		// Set the current limit
		result = pololu_smc_pointer->set_current_limit_in_milliamps(NEW_CURRENT_LIMIT_IN_MILLIAMPS);
		if (!result)
			ROS_INFO_STREAM("[MOTOR CONTROLLER] FAILED - Pololu SMC - set current limit NOT successful for I2C address " << pololu_smc_pointer->get_i2c_address() );

		// Check the current limit that was set
		uint16_t current_limit_value;
		result = pololu_smc_pointer->get_current_limit(&current_limit_value);
		if (result)
			ROS_INFO_STREAM("[MOTOR CONTROLLER] Pololu SMC - get current limit returned: " << current_limit_value << ", for I2C address " << pololu_smc_pointer->get_i2c_address() );
		else
			ROS_INFO_STREAM("[MOTOR CONTROLLER] FAILED - Pololu SMC - get current limit NOT successful for I2C address " << pololu_smc_pointer->get_i2c_address() );

		// Short sleep
		usleep(1000);


		// Send the max speed limit
		int max_speed_limit_response_code;
		result = pololu_smc_pointer->set_motor_limit_max_speed(NEW_MAX_SPEED_LIMIT, &max_speed_limit_response_code);
		if (!result)
			ROS_INFO_STREAM("[MOTOR CONTROLLER] FAILED - Pololu SMC - set max speed limit NOT successful with response code " << max_speed_limit_response_code << ", for I2C address " << pololu_smc_pointer->get_i2c_address() );

		// Check the max speed limit that was set
		uint16_t max_speed_limit_value;
		result = pololu_smc_pointer->get_max_speed_forward(&max_speed_limit_value);
		if (result)
			ROS_INFO_STREAM("[MOTOR CONTROLLER] Pololu SMC - get max speed limit returned: " << max_speed_limit_value << ", for I2C address " << pololu_smc_pointer->get_i2c_address() );
		else
			ROS_INFO_STREAM("[MOTOR CONTROLLER] FAILED - Pololu SMC - get max speed limit NOT successful for I2C address " << pololu_smc_pointer->get_i2c_address() );

		// Short sleep
		usleep(1000);


		// Set the max acceleration limit
		int max_accel_limit_response_code;
		result = pololu_smc_pointer->set_motor_limit_max_acceleration(NEW_MAX_ACCEL_LIMIT, &max_accel_limit_response_code);
		if (!result)
			ROS_INFO_STREAM("[MOTOR CONTROLLER] FAILED - Pololu SMC - set max acceleration limit NOT successful with response code " << max_accel_limit_response_code << " for I2C address " << pololu_smc_pointer->get_i2c_address() );

		// Check the max speed acceleration that was set
		uint16_t max_accel_limit_value;
		result = pololu_smc_pointer->get_max_acceleration_forward(&max_accel_limit_value);
		if (result)
			ROS_INFO_STREAM("[MOTOR CONTROLLER] Pololu SMC - get max acceleration limit returned: " << max_accel_limit_value << ", for I2C address " << pololu_smc_pointer->get_i2c_address() );
		else
			ROS_INFO_STREAM("[MOTOR CONTROLLER] FAILED - Pololu SMC - get max acceleration limit NOT successful for I2C address " << pololu_smc_pointer->get_i2c_address() );


		// Set the max deceleration limit
		int max_decel_limit_response_code;
		result = pololu_smc_pointer->set_motor_limit_max_deceleration(NEW_MAX_DECEL_LIMIT, &max_decel_limit_response_code);
		if (!result)
			ROS_INFO_STREAM("[MOTOR CONTROLLER] FAILED - Pololu SMC - set max deceleration limit NOT successful with response code " << max_decel_limit_response_code << " for I2C address " << pololu_smc_pointer->get_i2c_address() );

		// > Check the max speed deceleration that was set
		uint16_t max_decel_limit_value;
		result = pololu_smc_pointer->get_max_deceleration_forward(&max_decel_limit_value);
		if (result)
			ROS_INFO_STREAM("[MOTOR CONTROLLER] Pololu SMC - get max deceleration limit returned: " << max_decel_limit_value << ", for I2C address " << pololu_smc_pointer->get_i2c_address() );
		else
			ROS_INFO_STREAM("[MOTOR CONTROLLER] FAILED - Pololu SMC - get max deceleration limit NOT successful for I2C address " << pololu_smc_pointer->get_i2c_address() );

		// Short sleep
		usleep(1000);


		ROS_INFO_STREAM("[MOTOR CONTROLLER] Finished setting up the Pololu SMC with I2C address " << pololu_smc_pointer->get_i2c_address() );
	}


	// Enter a loop that continues while ROS is still running
	while (ros::ok())
	{
		// Reading of the current sensor to be implemented here


		// Spin once so that this node can service any
		// callbacks that this node has queued.
		ros::spinOnce();

		// Sleep for the specified loop rate
		loop_rate.sleep();
	} // END OF: "while (ros::ok())"

	// Close the I2C device
	bool close_success = m_i2c_driver.close_i2c_device();
	
	// Display the status
	if (!close_success)
		ROS_INFO_STREAM("[MOTOR CONTROLLER] FAILED to close I2C device named " << m_i2c_driver.get_device_name());
	else
		ROS_INFO_STREAM("[MOTOR CONTROLLER] Successfully closed device named " << m_i2c_driver.get_device_name());

	return 0;
}

int clip_pwm_duty_cycle(int pwm_duty_cycle)
{
	if (pwm_duty_cycle < -100)
		pwm_duty_cycle = -100;
	if (pwm_duty_cycle > 100)
		pwm_duty_cycle = 100;

	return pwm_duty_cycle;
}
