#ifndef MOTIONPLANHEADER_H
#define MOTIONPLANHEADER_H

#include "ros/ros.h"
#include "amr/amr.h"
#include <math.h>
#include <stdlib.h>
#include <vector>
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Point32.h"


struct discreteParam
{
    float prev      = 0.0f;
    float current   = 0.0f;

    void updateNewValue()
    {
        prev = current;
    }
};

enum FSM_State: int
{
    DEFAULT = 0,
    LOAD    = 1,
    UNLOAD  = 2,
};

struct PIDController
{
    float KP = 0.0f;
    float KI = 0.0f;
    float KD = 0.0f;

    PIDController(float kp, float ki, float kd)
    {
        this->KP = kp;
        this->KI = ki;
        this->KD = kd;
    }

    // float discretePI(discreteParam u, discreteParam error)
    // {
    //     return u.prev + error.current*(this->KP + this->KI*AMR_TIMER_4_SYSTEM_CONTROL_IN_SEC/2) + (this->KI*AMR_TIMER_4_SYSTEM_CONTROL_IN_SEC/2 - this->KP)*error.prev;
    // }

    // float discretePI_FF(discreteParam u, discreteParam error, discreteParam ref)
    // {
    //     return u.prev + ref.current - ref.prev + error.current*(this->KP + this->KI*AMR_TIMER_4_SYSTEM_CONTROL_IN_SEC/2) + (this->KI*AMR_TIMER_4_SYSTEM_CONTROL_IN_SEC/2 - this->KP)*error.prev;
    // }

    float discreteP(discreteParam error)
    {
        return error.current*this->KP;
    }

};

struct pathStruct
{
    geometry_msgs::Polygon path = geometry_msgs::Polygon();
    int noPoints = 0;

    void importPolygonMSG(const geometry_msgs::Polygon& msg)
    {
        this->path          = msg;
        this->noPoints      = this->path.points.size();
    }
    void isAvailable(std::string name)
    {
        if (this->path.points.empty())
        {
            ROS_INFO_STREAM("[" << name << "]"<< " EMPTY PATH !!!");
        }
        else
        {
            ROS_INFO_STREAM("[" << name << "]"<<" VALID PATH !!!");
        }
    }
};

AmrPose currentPose = AmrPose();
AmrPose refPose = AmrPose();
MotorAngularVelocity RotationAngleMsg;
std::string Robot_State = amr_state::IDLE;

#endif