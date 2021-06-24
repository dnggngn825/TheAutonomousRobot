/* ==================================================
AUTHORSHIP STATEMENT
The University of Melbourne
School of Engineering
ELEN90090: Autonomous Clinic Systems
Author: Quang Trung Le (987445)
================================================== */


// ==================================================
// Including library
// ==================================================
#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/UInt16.h"
#include "amr/amr.h"
#include <bitset>
using namespace asclinic_pkg;


// ==================================================
// Constants/Variables/Functions 
// ==================================================
const int MAX_PULSE_WIDTH_IN_US = 2500;
const int MIN_PULSE_WIDTH_IN_US = 500;


I2C_Driver m_i2c_driver (AMR_I2C_DEVICE_NAME); // Hardware setup for this node

const uint8_t m_pca9685_address = 0x42; // > For the PCA9685 PWM Servo Driver driver
PCA9685 m_pca9685_servo_driver (&m_i2c_driver, m_pca9685_address);


void subscriberCallback4ServoControlSignal(const ServoPulseWidth& msg);
uint16_t clip_pulse_width_in_us(uint16_t pulse_width_in_us);
void initialise_servo(float frequency_in_hz);
void open_i2c_driver();
void close_i2c_driver();



// ==================================================
// MAIN Program
// ==================================================
int main(int argc, char* argv[])
{
	// Initialise the node
	ros::init(argc, argv, amr_node::SERVO_DYNAMICS);
	ros::NodeHandle nd;

	ros::Subscriber subscriber_4_servo_ctl_signal = nd.subscribe(amr_topic::SERVO_CONTROL_SIGNAL, 1, subscriberCallback4ServoControlSignal);

	open_i2c_driver();

	// SET THE CONFIGURATION OF THE SERVO DRIVER
	float frequency_in_hz = 50.0;
	initialise_servo(frequency_in_hz);

	
	ros::spin();
	close_i2c_driver();

	return 0;
}



// ==================================================
// CALLBACK Functions
// ==================================================
void subscriberCallback4ServoControlSignal(const ServoPulseWidth& msg)
{
	uint8_t channel = msg.channel;
	uint16_t pulse_width_in_us = msg.pulse_width_in_microseconds;

	// Display the message received
	ROS_INFO_STREAM("Message receieved for servo with channel = " << static_cast<int>(channel) << ", and pulse width [us] = " << static_cast<int>(pulse_width_in_us) );

	pulse_width_in_us = clip_pulse_width_in_us(pulse_width_in_us);


	// Call the function to set the desired pulse width
	bool result = m_pca9685_servo_driver.set_pwm_pulse_in_microseconds(15, pulse_width_in_us);
	if (!result) // Display if an error occurred
		ROS_INFO_STREAM("FAILED to set pulse width for servo at channel " << static_cast<int>(channel) );
}


// ==================================================
// Other Functions
// ==================================================
void initialise_servo(float frequency_in_hz)
{
	// Call the Servo Driver initialisation function
	bool result_servo_init = m_pca9685_servo_driver.initialise_with_frequency_in_hz(frequency_in_hz, false);

	// Display if an error occurred
	if (!result_servo_init)
	{
		ROS_INFO_STREAM("FAILED - while initialising servo driver with I2C address " << static_cast<int>(m_pca9685_servo_driver.get_i2c_address()) );
	}
}


uint16_t clip_pulse_width_in_us(uint16_t pulse_width_in_us)
{
	if (pulse_width_in_us > 0)
	{
		if (pulse_width_in_us < MIN_PULSE_WIDTH_IN_US)
			pulse_width_in_us = MIN_PULSE_WIDTH_IN_US;
		if (pulse_width_in_us > MAX_PULSE_WIDTH_IN_US)
			pulse_width_in_us = MAX_PULSE_WIDTH_IN_US;
	}

	return pulse_width_in_us;
}


/**
 * @brief Open the I2C device.
 * 
 */
void open_i2c_driver()
{
	bool is_motor_open_success = m_i2c_driver.open_i2c_device();
	if (!is_motor_open_success)
		ROS_INFO_STREAM("[MOTOR DYNAMICS] FAILED to open I2C device named " << m_i2c_driver.get_device_name());
	else
		ROS_INFO_STREAM("[MOTOR DYNAMICS] Successfully opened named " << m_i2c_driver.get_device_name() << ", with file descriptor = " << m_i2c_driver.get_file_descriptor());
}


/**
 * @brief Close the I2C device.
 * 
 */
void close_i2c_driver()
{
	bool close_success = m_i2c_driver.close_i2c_device();
	
	// Display the status
	if (!close_success)
		ROS_INFO_STREAM("[MOTOR DYNAMICS] FAILED to close I2C device named " << m_i2c_driver.get_device_name());
	else
		ROS_INFO_STREAM("[MOTOR DYNAMICS] Successfully closed device named " << m_i2c_driver.get_device_name());
}


