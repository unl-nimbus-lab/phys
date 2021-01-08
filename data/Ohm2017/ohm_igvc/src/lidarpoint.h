#ifndef LIDARPOINT_H
#define LIDARPOINT_H

class LidarPoint
{
    public:
        double x;
        double y;
        double distanceFromRobot;
        
        LidarPoint(int X = 0, int Y = 0, double distance = 0)
        {
            x = X;
            y = Y;
            distanceFromRobot = distance;
        }
};

#endif