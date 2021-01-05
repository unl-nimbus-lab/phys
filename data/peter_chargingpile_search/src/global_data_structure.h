
#pragma once

#include <ros/ros.h>

namespace weighted_fit {
typedef struct
{
    double x;
    double y;
} iPoint;


typedef struct{
    double a;   //y = a*x + b
    double b;
    double Rho; // 该段直线的倾角
    iPoint startPoint;
    iPoint endPoint;
    double standardDeviation;
} LinePara;


}




namespace peter_chargingpile_search {
struct PointInfo
{
    double x;
    double y;
    size_t index;
    double angle;
    double range;
};

struct XYPoint
{
    double x;
    double y;
};


struct PointWithTimeStamp
{
    double x;
    double y;
    double z;
    double theta;   //angle with x-axis
    ros::Time timestamp;

};

struct UpdateDataPacket
{
    bool chargeOrderFlag;
    bool powerStatusFlag;
    PointWithTimeStamp objPosition;
    bool outOfTimeFlag;
};


}
