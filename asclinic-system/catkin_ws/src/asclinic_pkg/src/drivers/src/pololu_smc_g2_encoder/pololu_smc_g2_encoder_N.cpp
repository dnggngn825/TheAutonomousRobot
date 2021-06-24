/* ==================================================
AUTHORSHIP STATEMENT
The University of Melbourne
School of Engineering
ELEN90090: Autonomous Clinic Systems
Author: Quang Trung Le (987445)
================================================== */


#include "pololu_smc_g2_encoder/pololu_smc_g2_encoder.h"
#include "amr/amr_connection/amr_connection_constants.h"

#define _USE_MATH_DEFINES // for C
#include <math.h>


#include "ros/ros.h"
#include <ros/package.h>


Pololu_SMC_G2_Encoder::Pololu_SMC_G2_Encoder()
{
	this->angular_velocity_rpm = 0;
	this->dir_channel_a2b = 0;
	this->dir_channel_b2a = 0;

	this->chip = AMR_GPIO_CHIP_NAME;
    this->line_channel_a = 0;
    this->line_channel_b = 0;
}

Pololu_SMC_G2_Encoder::Pololu_SMC_G2_Encoder(int dir_channel_a2b)
{
	Pololu_SMC_G2_Encoder();
	this->set_dir_channel_a2b(dir_channel_a2b);

	for (int i=0; i<sizeof(this->STATE_DIRECTION_DEFAULT); i++)
	{
		this->state_directions[i] = this->STATE_DIRECTION_DEFAULT[i]*dir_channel_a2b;
	}
}

// ==================================================
// Setter and getter
// ==================================================
/**
 * @brief [Setter] Set angular velocity in rpm.
 * 
 * @param angular_velocity_rpm Angular velocity (rpm).
 */
void Pololu_SMC_G2_Encoder::set_angular_velocity_rpm(float angular_velocity_rpm)
{
	this->angular_velocity_rpm = angular_velocity_rpm;
}

/**
 * @brief [Getter] Get angular velocity in rpm.
 * 
 * @return float 
 */
float Pololu_SMC_G2_Encoder::get_angular_velocity_rpm()
{
    return this->angular_velocity_rpm;
}



/* 
void Pololu_SMC_G2_Encoder::set_cw_direction(int cw_direction)
{
	if (cw_direction>1)
		cw_direction = 1;
	else if (cw_direction<-1)
		cw_direction = -1;
	
	this->cw_direction = cw_direction;
	this->ccw_direction = cw_direction*(-1);
}

int Pololu_SMC_G2_Encoder::get_cw_direction()
{
	return this->cw_direction;
}
 */

/**
 * @brief [Setter] Set Clockwise direction of encoder.
 * 
 * @param dir_channel_a2b +1/-1 equivalent with cw/ccw direction.
 */
void Pololu_SMC_G2_Encoder::set_dir_channel_a2b(int dir_channel_a2b)
{
	if (dir_channel_a2b>1)
		dir_channel_a2b = 1;
	else if (dir_channel_a2b<-1)
		dir_channel_a2b = -1;
	
	this->dir_channel_a2b = dir_channel_a2b;
	this->dir_channel_b2a = dir_channel_a2b*(-1);
}

/**
 * @brief [Getter] Get the Clockwise direction of encoder.
 * 
 * @return [int] +1/-1: Equivalent to forward/backward direction.
 */
int Pololu_SMC_G2_Encoder::get_dir_channel_a2b()
{
	return this->dir_channel_a2b;
}


/**
 * @brief [Setter] Set counter-clockwise direction of encoder.
 * 
 * @param cw_direction +1/-1 equivalent with forward/backward direction.
 */
void Pololu_SMC_G2_Encoder::set_dir_channel_b2a(int dir_channel_b2a)
{
	if (dir_channel_b2a>1)
		dir_channel_b2a = 1;
	else if (dir_channel_b2a<-1)
		dir_channel_b2a = -1;
	
	this->dir_channel_b2a = dir_channel_b2a;
	this->dir_channel_a2b = dir_channel_b2a*(-1);
}

/**
 * @brief [Getter] Get the counter-clockwise direction of encoder.
 * 
 * @return [int] +1/-1: Equivalent to forward/backward direction.
 */
int Pololu_SMC_G2_Encoder::get_dir_channel_b2a()
{
	return this->dir_channel_b2a;
}


// ==================================================
// Other Functions
// ==================================================
/**
 * @brief Set angular velocity (rpm) (equal to a setter).
 * 
 * @param encoder_counter The number of encoder counter in sampling period (pulses).
 * @param sampling_time_in_sec Sampling period (s).
 */
void Pololu_SMC_G2_Encoder::set_angular_velocity_rpm(int encoder_counter, float sampling_time_in_sec)
{
	float angular_velocity_rpm = (encoder_counter*1.0f/POLOLU_SMC_G2_ENCODER_RESOLUTION)/sampling_time_in_sec*60.0f;
	this->angular_velocity_rpm = (angular_velocity_rpm);
}

/**
 * @brief Get angular velocity in revolution per second.
 * 
 * @return float 
 */
float Pololu_SMC_G2_Encoder::get_angular_velocity_rps()
{
	return (this->angular_velocity_rpm/60.0f);
}

/**
 * @brief Get angular velocity in radian per second.
 * 
 * @return float 
 */
float Pololu_SMC_G2_Encoder::get_angular_velocity_rdps()
{
	return (2*M_PI*this->get_angular_velocity_rps());
}

/**
 * @brief Get angular velocity in pwm
 * 
 * @return float 
 */
float Pololu_SMC_G2_Encoder::get_angular_velocity_pwm()
{
	return round(100*(this->angular_velocity_rpm/POLOLU_SMC_G2_MOTOR_MAX_ANGULAR_VELOCITY_RPM_W_GEARBOX));
}

/**
 * @brief Get rotation angle in sampling period (s).
 * 
 * @param sampling_period_in_sec Sampling period (s).
 * @return float 
 */
float Pololu_SMC_G2_Encoder::get_rotation_angle_rad(float sampling_period_in_sec)
{
	return (this->get_angular_velocity_rdps() * sampling_period_in_sec);
}

/**
 * @brief Get rotation angle in sampling period (s)
 * @param sampling_period_in_sec Sampling time in second (s)
 * @return float 
 */
float Pololu_SMC_G2_Encoder::get_rotation_angle_rev(float sampling_period_in_sec)
{
	return (this->get_rotation_angle_rad(sampling_period_in_sec) / (2*M_PI));
}

/**
 * @brief Get line of channel A of encoder.
 * 
 * @return int 
 */
int Pololu_SMC_G2_Encoder::get_line_channel_a()
{
	return this->line_channel_a;
}

/**
 * @brief Get line of channel B of encoder.
 * 
 * @return int 
 */
int Pololu_SMC_G2_Encoder::get_line_channel_b()
{
	return this->line_channel_b;
}



// ==================================================
// Functions dealing with gpiod_chip, channel A, channel B
// ==================================================
/**
 * @brief Set the GPIO chip, which connects with the Encoder.
 * 
 * @param chip chip name.
 * @param gpiod_chip gpiod chip.
 */
void Pololu_SMC_G2_Encoder::set_gpiod_chip(const char* chip, struct gpiod_chip *gpiod_chip)
{
	this->chip = chip;
	this->gpiod_chip = gpiod_chip;
}

/**
 * @brief Set the default GPIO chip, which connects with the Encoder.
 * 
 */
void Pololu_SMC_G2_Encoder::set_gpiod_chip()
{
	this->chip = AMR_GPIO_CHIP_NAME;

    struct gpiod_chip *gpiod_chip = gpiod_chip_open(this->chip);
	this->gpiod_chip = gpiod_chip;
}

/**
 * @brief Set the GPIO line channels, which connect with channel A and channel B of encoder.
 * 
 * @param line_channel_a Line of channel A.
 * @param line_channel_b Line of channel B.
 */
void Pololu_SMC_G2_Encoder::set_gpiod_line_channel(int line_channel_a, int line_channel_b)
{
	this->line_channel_a = line_channel_a;
	this->line_channel_b = line_channel_b;

	this->gpiod_line_channel_a = gpiod_chip_get_line(this->gpiod_chip, this->line_channel_a);
	this->gpiod_line_channel_b = gpiod_chip_get_line(this->gpiod_chip, this->line_channel_b);
}

/**
 * @brief Set line request event of encoder channel A.
 * 
 * @param line_request_event Rising Edge/Falling Edge/Both Edges.
 */
void Pololu_SMC_G2_Encoder::set_gpiod_line_request_events(Pololu_SMC_G2_Encoder::Line_Request_Event line_request_event)
{
	switch(line_request_event) 
	{
		case Pololu_SMC_G2_Encoder::Line_Request_Event::RISING_EDGE:
			gpiod_line_request_rising_edge_events(this->gpiod_line_channel_a, "foobar");
			break;
		case Pololu_SMC_G2_Encoder::Line_Request_Event::FALLING_EDGE:
			gpiod_line_request_falling_edge_events(this->gpiod_line_channel_a, "foobar");
			break;
		case Pololu_SMC_G2_Encoder::Line_Request_Event::BOTH_EDGES:
		default:
			gpiod_line_request_both_edges_events(this->gpiod_line_channel_a, "foobar");
			break;
	}
}

/**
 * @brief Count number of encoder pulses, using no debounce. Note: counter is tracked to be in the same direction with CW.
 * 
 * @return [int] motor direction (+1 CW / -1 CCW)  
 */
int Pololu_SMC_G2_Encoder::monitor_encoder_trigger_falling_edge_wo_debounce(struct gpiod_line_event event) // minimal version
{
	int motor_direction = 0;

	int returned_wait_flag = gpiod_line_event_wait(this->gpiod_line_channel_a, &this->TIME_OUT);
	if (returned_wait_flag == 1)
	{
		// Identify rotation direction
		int state_channel_b = gpiod_ctxless_get_value(this->chip, this->line_channel_b, false, "foobar");
		if (state_channel_b == Line_State::LOW)
			motor_direction = this->dir_channel_b2a;
		else if (state_channel_b == Line_State::HIGH)
			motor_direction = this->dir_channel_a2b;
	}

	return motor_direction;
}

/* int Pololu_SMC_G2_Encoder::monitor_encoder_trigger_falling_edge_wo_debounce(struct gpiod_line_event event)
{
	int motor_direction = 0;

	int returned_wait_flag = gpiod_line_event_wait(this->gpiod_line_channel_a, &this->TIME_OUT);
	switch (returned_wait_flag) // Respond based on the the return flag
	{
		case 1: // Event occurred:
		{
			int returned_read_flag = gpiod_line_event_read(this->gpiod_line_channel_a,&event); // Read the pending event on the GPIO line

			switch (returned_read_flag)
			{
				case 0: // Event read correctly
				{
					// Identify rotation direction
					int state_channel_b = gpiod_ctxless_get_value(this->chip, this->line_channel_b, false, "foobar");
					if (state_channel_b == Line_State::LOW)
						motor_direction = this->dir_channel_b2a;
					else if (state_channel_b == Line_State::HIGH)
						motor_direction = this->dir_channel_a2b;
					
					break;
				}

				case -1:
				default:
				{
					ROS_INFO_STREAM("[ENCODER READER] gpiod_line_event_read returned an unrecognised status, return_flag =  " << returned_read_flag );
					break;
				}
			} // END OF: "switch (returned_read_flag)"
			break;
		}
		
		case 0: // Time out occurred
		{
			break;
		}

		case -1: // Error occurred
		{
			// Display the status
			ROS_INFO("[ENCODER READER] gpiod_line_event_wait returned the status that an error occurred");
			break;
		}

		default:
		{
			// Display the status
			ROS_INFO_STREAM("[ENCODER READER] gpiod_line_event_wait returned an unrecognised status, return_flag =  " << returned_wait_flag );
			break;
		}

	} // END OF: "switch (returned_wait_flag)"

	return motor_direction;
}
 */


/**
 * @brief Count number of encoder pulses, using time interval debounce.
 * 
 * @return [int] motor direction (+1 forward / -1 backward)  
 */
int Pololu_SMC_G2_Encoder::monitor_encoder_trigger_falling_edge_w_time_debounce(struct gpiod_line_event event)
{
	int motor_direction = 0;

	int channel_a_wait_flag = gpiod_line_event_wait(this->gpiod_line_channel_a, &this->TIME_OUT);
	switch (channel_a_wait_flag) // Respond based on the the return flag
	{
		case 1: // Event occurred:
		{
			int channel_a_read_flag = gpiod_line_event_read(this->gpiod_line_channel_a,&event); // Read the pending event on the GPIO line
			// int pre_state_channel_a = gpiod_ctxless_get_value(this->chip, this->line_channel_a, false, "foobar");
			usleep(POLOLU_SMC_G2_ENCODER_DEBOUNCE_INTERVAL_SESC_W_GEARBOX);
			int new_state_channel_a = gpiod_ctxless_get_value(this->chip, this->line_channel_a, false, "foobar");

			if (new_state_channel_a == Line_State::LOW)
			{
				switch (channel_a_read_flag)
				{
					case 0: // Event read correctly
					{
						// Identify rotation direction
						int state_channel_b = gpiod_ctxless_get_value(this->chip, this->line_channel_b, false, "foobar");
						if (state_channel_b == Line_State::LOW)
							motor_direction = this->dir_channel_b2a;
						else if (state_channel_b == Line_State::HIGH)
							motor_direction = this->dir_channel_a2b;
						
						break;
					}

					case -1:
					default:
					{
						ROS_INFO_STREAM("[ENCODER READER] gpiod_line_event_read returned an unrecognised status, return_flag =  " << channel_a_read_flag );
						break;
					}
				} // END OF: "switch (returned_read_flag)"
			}
			
			break;
		}
		
		case 0: // Time out occurred
		{
			break;
		}

		case -1: // Error occurred
		{
			// Display the status
			ROS_INFO("[ENCODER READER] gpiod_line_event_wait returned the status that an error occurred");
			break;
		}

		default:
		{
			// Display the status
			ROS_INFO_STREAM("[ENCODER READER] gpiod_line_event_wait returned an unrecognised status, return_flag =  " << channel_a_wait_flag );
			break;
		}

	} // END OF: "switch (returned_wait_flag)"

	return motor_direction;
}

/**
 * @brief Count number of encoder pulses, using no debounce. The #counter = 2 * #pulses.
 * 
 * @return [int] motor direction (+1 forward / -1 backward)  
 */
int Pololu_SMC_G2_Encoder::monitor_encoder_trigger_both_edges_wo_debounce(struct gpiod_line_event event)
{
	int motor_direction = 0;

	int returned_wait_flag = gpiod_line_event_wait(this->gpiod_line_channel_a, &this->TIME_OUT);
	switch (returned_wait_flag) // Respond based on the the return flag
	{
		case 1: // Event occurred:
		{
			int returned_read_flag = gpiod_line_event_read(this->gpiod_line_channel_a,&event); // Read the pending event on the GPIO line

			switch (returned_read_flag)
			{
				case 0: // Event read correctly
				{
					int state_channel_b = gpiod_ctxless_get_value(this->chip, this->line_channel_b, false, "foobar");

					// Identify rotation direction
					if (((event.event_type == GPIOD_LINE_EVENT_RISING_EDGE) && (state_channel_b == Line_State::LOW)) ||
						((event.event_type == GPIOD_LINE_EVENT_FALLING_EDGE) && (state_channel_b == Line_State::HIGH)))  // CW
						motor_direction = this->dir_channel_a2b;
					else if (((event.event_type == GPIOD_LINE_EVENT_FALLING_EDGE) && (state_channel_b == Line_State::LOW)) ||
							((event.event_type == GPIOD_LINE_EVENT_RISING_EDGE) && (state_channel_b == Line_State::HIGH)))  // CCW
						motor_direction = this->dir_channel_b2a;

					break;
				}

				case -1:
				default:
				{
					ROS_INFO_STREAM("[ENCODER READER] gpiod_line_event_read returned an unrecognised status, return_flag =  " << returned_read_flag );
					break;
				}
			} // END OF: "switch (returned_read_flag)"
			break;
		}
		
		case 0: // Time out occurred
		{
			break;
		}

		case -1: // Error occurred
		{
			// Display the status
			ROS_INFO("[ENCODER READER] gpiod_line_event_wait returned the status that an error occurred");
			break;
		}

		default:
		{
			// Display the status
			ROS_INFO_STREAM("[ENCODER READER] gpiod_line_event_wait returned an unrecognised status, return_flag =  " << returned_wait_flag );
			break;
		}

	} // END OF: "switch (returned_wait_flag)"

	return motor_direction;
}

/**
 * @brief Count number of encoder pulses, using digital filter debounce (count 12 periods).
 * 
 * @return [int] motor direction (+1 forward / -1 backward)  
 */
int Pololu_SMC_G2_Encoder::monitor_encoder_polling_w_df_debounce()
{
	int motor_direction = 0;
	int state_channel_a = gpiod_ctxless_get_value(this->chip, this->line_channel_a, false, "foobar");

	this->df_debounce_state = (this->df_debounce_state<<1) | state_channel_a | 0xe000;

	if (this->df_debounce_state == 0xf000) // debounce in 12 periods
	{
		int state_channel_b = gpiod_ctxless_get_value(this->chip, this->line_channel_b, false, "foobar");
		if (state_channel_b == Line_State::HIGH)
			motor_direction = this->dir_channel_a2b;
		else if (state_channel_b == Line_State::LOW)
			motor_direction = this->dir_channel_b2a;
	}
	return motor_direction;
}

/**
 * @brief Count number of encoder pulses, using state waves debounce (track states of 2 channels).
 * 
 * @return [int] motor direction (+1 forward / -1 backward)  
 */
int Pololu_SMC_G2_Encoder::monitor_encoder_polling_w_sw_debounce()
{
	int motor_direction = 0;
	int state_channel_a = (gpiod_ctxless_get_value(this->chip, this->line_channel_a, false, "foobar"));
	int state_channel_b = (gpiod_ctxless_get_value(this->chip, this->line_channel_b, false, "foobar"));

	// static int sw_pre_state_int = 0;
	

	static bool start_monitor = true;
	// static int sw_states_tracker = 0;
	if (start_monitor)
	{
		// sw_pre_state_int = (state_channel_a*2 + state_channel_b)%4;
		// sw_pre_state = (state_channel_a*2 + state_channel_b)%4;
		sw_pre_state = ((state_channel_a<<1) + (state_channel_b))%4;
		start_monitor = false;
	}	

	// sw_new_state = (sw_pre_state*4+state_channel_a*2+state_channel_b)%16;
	sw_new_state = ((sw_pre_state<<2) + (state_channel_a<<1) + (state_channel_b))%16;
	sw_states_tracker += this->STATE_DIRECTION_DEFAULT[sw_new_state];
	sw_pre_state = sw_new_state%4;

	// int sw_new_state_int = (sw_pre_state_int*4+state_channel_a*2+state_channel_b)%16;
	// sw_states_tracker += this->STATE_DIRECTION_DEFAULT[sw_new_state_int];
	// sw_pre_state_int = sw_new_state_int%4;

	if (sw_states_tracker == NUM_CHANNEL_STATE_TRACKER || sw_states_tracker == -NUM_CHANNEL_STATE_TRACKER)
	{
		motor_direction = sw_states_tracker/NUM_CHANNEL_STATE_TRACKER * this->dir_channel_a2b;
		sw_states_tracker = 0;
	}

	return motor_direction;
}



int Pololu_SMC_G2_Encoder::get_channel_event_type(struct gpiod_line *gpiod_line_channel, struct gpiod_line_event event)
{
	int event_type = -1;

	int returned_wait_flag = gpiod_line_event_wait(gpiod_line_channel, &this->TIME_OUT);
	switch (returned_wait_flag) // Respond based on the the return flag
	{
		case 1: // Event occurred:
		{
			int returned_read_flag = gpiod_line_event_read(gpiod_line_channel,&event); // Read the pending event on the GPIO line

			switch (returned_read_flag)
			{
				case 0: // Event read correctly
				{
					event_type = event.event_type;
					break;
				}

				case -1:
				default:
				{
					ROS_INFO_STREAM("[ENCODER READER] gpiod_line_event_read returned an unrecognised status, return_flag =  " << returned_read_flag );
					break;
				}
			} // END OF: "switch (returned_read_flag)"
			break;
		}
		
		case 0: // Time out occurred
		{
			break;
		}

		case -1: // Error occurred
		{
			// Display the status
			ROS_INFO("[ENCODER READER] gpiod_line_event_wait returned the status that an error occurred");
			break;
		}

		default:
		{
			// Display the status
			ROS_INFO_STREAM("[ENCODER READER] gpiod_line_event_wait returned an unrecognised status, return_flag =  " << returned_wait_flag );
			break;
		}

	} // END OF: "switch (returned_wait_flag)"

	return event_type;
}

int Pololu_SMC_G2_Encoder::get_channel_a_event_type(struct gpiod_line_event event)
{
	return Pololu_SMC_G2_Encoder::get_channel_event_type(this->gpiod_line_channel_a, event);
}

int Pololu_SMC_G2_Encoder::get_channel_b_event_type(struct gpiod_line_event event)
{
	return Pololu_SMC_G2_Encoder::get_channel_event_type(this->gpiod_line_channel_b, event);
}
