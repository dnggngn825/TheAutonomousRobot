/* ==================================================
AUTHORSHIP STATEMENT
The University of Melbourne
School of Engineering
ELEN90090: Autonomous Clinic Systems
Author: Quang Trung Le (987445)
================================================== */


#include "std_msgs/UInt32.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "asclinic_pkg/AmrPose.h"
#include "asclinic_pkg/MotorRotationAngle.h"
#include "asclinic_pkg/MotorAngularVelocityPWM.h"
#include "asclinic_pkg/MotorAngularVelocity.h"
#include "asclinic_pkg/ServoPulseWidth.h"

using namespace asclinic_pkg;

#ifndef AMR_MSGS_H
#define AMR_MSGS_H


namespace amr_msgs
{
    typedef std_msgs::UInt32    TimerID;
    typedef std_msgs::Int16     EncCounter;
    typedef std_msgs::String    StopIndicator;
    typedef std_msgs::Int16     EncLineState;
    typedef std_msgs::Int16     GeneralMsg;
    typedef std_msgs::Int16     ProxSensorState;
    typedef std_msgs::Int16     RobotState;
    typedef std_msgs::Int16     RobotMovementType;
    
}

#endif 
