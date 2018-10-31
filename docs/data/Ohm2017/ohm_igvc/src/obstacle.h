#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "lidarpoint.h"

class Obstacle
{
    public:
        double x;
        double y;
        double distance;
        int objStartIndex;
        int objEndIndex;
        double obsRoughSize;
        double obsLineSize;
        LidarPoint startPoint;
        LidarPoint endPoint;
        Obstacle (){

        }
};

#endif