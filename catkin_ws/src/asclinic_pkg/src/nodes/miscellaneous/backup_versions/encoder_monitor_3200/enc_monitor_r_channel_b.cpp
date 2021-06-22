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
#include <gpiod.h>
#include "amr/amr.h"
using namespace asclinic_pkg;


// ==================================================
// Constants/Variables/Functions 
// ==================================================
// Variables and Functions
struct gpiod_chip *gpiod_chip = gpiod_chip_open(AMR_GPIO_CHIP_NAME); // Open the GPIO chip
struct gpiod_line_event gpiod_line_event;

ros::Publisher publisher_enc_state;

Pololu_SMC_G2_Encoder encoder = Pololu_SMC_G2_Encoder(POLOLU_SMC_G2_ENCODER_DIR_CHANNEL_A2B_MOTOR_R);



// ==================================================
// MAIN Program
// ==================================================
int main(int argc, char* argv[])
{
	// Initialise the node
	ros::init(argc, argv, amr_node::ENC_MONITOR_R_CHANNEL_B);
	ros::NodeHandle nd;

    publisher_enc_state = nd.advertise<amr_msgs::EncLineState>(amr_topic::ENC_R_CHANNEL_B_STATE, 10, false);

	// ENC setup
	encoder.set_gpiod_chip();
	encoder.set_gpiod_line_channel(AMR_ENCODER_PIN_CHANNEL_A_MOTOR_R, AMR_ENCODER_PIN_CHANNEL_B_MOTOR_R);
	encoder.set_gpiod_line_request_events(Pololu_SMC_G2_Encoder::Line_Request_Event::BOTH_EDGES);


	amr_msgs::EncLineState msg;
	msg.data = gpiod_ctxless_get_value(AMR_GPIO_CHIP_NAME, AMR_ENCODER_PIN_CHANNEL_B_MOTOR_R, false, "foobar");
	publisher_enc_state.publish(msg);
	
	// Loop
	while (ros::ok())
	{
        int event_type = encoder.get_channel_b_event_type(gpiod_line_event);
        if (event_type >0)
		{
			msg.data = event_type;
			publisher_enc_state.publish(msg);
		}
            
		ros::spinOnce();
	}

	gpiod_chip_close(gpiod_chip); // Close the GPIO chip

	return 0;
}

