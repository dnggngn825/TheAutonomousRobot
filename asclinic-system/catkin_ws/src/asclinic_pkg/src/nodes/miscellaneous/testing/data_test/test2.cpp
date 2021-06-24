#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt32.h"

#include <bitset>
#include "amr/amr.h"

using namespace asclinic_pkg;

bool show_ros_info_flag;
int mode = 0;

#include <iostream>
#include <fstream>
void write_data();

void subscriberCallback(const std_msgs::UInt32& msg)
{
    ROS_INFO_STREAM("Number: "<<msg.data);
}

int main(int argc,  char* argv[])
{
    ros::init(argc, argv, "test_1");
    ros::NodeHandle nd;
    
    
    ros::Publisher publisher = nd.advertise<std_msgs::UInt32>("publisher", 10, false);  
    ros::Subscriber subscriber = nd.subscribe("publisher", 1, subscriberCallback);  

    ros::Rate loop_rate(1);

    
    nd.getParam("show_ros_info_flag", show_ros_info_flag);
    nd.getParam("mode", mode);

    int encoder_counter = 26;
    float sampling_time_in_sec = AMR_TIMER_4_SYSTEM_CONTROL_IN_SEC;
    float angular_velocity_rpm = (encoder_counter*1.0/POLOLU_SMC_G2_ENCODER_RESOLUTION)/sampling_time_in_sec*60.0;
    float angular_velocity_pwm_pre = 100.0f*angular_velocity_rpm/POLOLU_SMC_G2_MOTOR_MAX_ANGULAR_VELOCITY_RPM_W_GEARBOX;
    int angular_velocity_pwm = round(angular_velocity_pwm_pre);



    if (show_ros_info_flag)
    {
        ROS_INFO_STREAM("sampling_time_in_sec "<< sampling_time_in_sec);
        ROS_INFO_STREAM("angular_velocity_rpm "<< angular_velocity_rpm);
    }

    ROS_INFO_STREAM("mode: "<<mode+1);

    // write_data();
}




void write_data()
{
    ROS_INFO("write_function");
	std::ofstream myfile;
	// myfile.open ("/home/toothless/asclinic-system/catkin_ws/src/asclinic_pkg/src/nodes/miscellaneous/testing/data_test/example.csv");
	myfile.open ("example.csv");
    myfile <<   (std::to_string(1.12) 
                + "," + std::to_string(2.22526) 
                + "\n");
	ROS_INFO("write_function - about to close");
	myfile.close();  	
}
