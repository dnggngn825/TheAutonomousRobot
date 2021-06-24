#include "ros/ros.h"
#include <ros/package.h>

#include <bitset>
#include "amr/amr.h"
#include "asclinic_pkg/MotorsVoltage.h"

using namespace asclinic_pkg;



void subscriberCallback(const MotorsVoltage& msg)
{
    ROS_INFO_STREAM("Number: "<<msg.motor_l.volt);
}

int main(int argc,  char* argv[])
{
    ros::init(argc, argv, "test_3");
    ros::NodeHandle nd;
    
    
    ros::Publisher publisher = nd.advertise<MotorsVoltage>("publisher", 10, false);  
    ros::Subscriber subscriber = nd.subscribe("publisher", 1, subscriberCallback);  

    ros::Rate loop_rate(1);

    int i = 0;
    while(ros::ok())
    {   
        i++;
        MotorsVoltage msg;
        msg.motor_l.volt = i;
        publisher.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }


}