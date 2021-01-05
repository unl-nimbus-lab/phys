#ifndef NODE_H
#define NODE_H

#include "MapMaker.h"
#include <string>

const int dir = 8;

class Node
{
    // current position
    int xPos;
    int yPos;
    // total distance already travelled to reach the node
    int level;
    // priority=level+remaining distance estimate
    int priority;  // smaller: higher priority

    public:
        Node(int xp, int yp, int d, int p)
            {xPos=xp; yPos=yp; level=d; priority=p;}

        int GetxPos() const {return xPos;}
        int GetyPos() const {return yPos;}
        int GetLevel() const {return level;}
        int GetPriority() const {return priority;}

        void UpdatePriority(const int & xDest, const int & yDest)
        {
             priority = level + Estimate(xDest, yDest) * 10; //A*
        }

        // give better priority to going stright instead of diagonally
        void NextLevel(const int & i) // i: direction
        {
             level += (dir == 8 ? (i % 2 == 0 ? 10 : 14) : 10);
        }

        // Estimation function for the remaining distance to the goal.
        const int & Estimate(const int & xDest, const int & yDest) const
        {
            static int xd, yd, d;
            xd = xDest-xPos;
            yd = yDest-yPos;

            // Euclidian Distance
            d=static_cast<int>(sqrt(xd*xd+yd*yd));

            // Manhattan distance
            //d=abs(xd)+abs(yd);

            // Chebyshev distance
            //d=max(abs(xd), abs(yd));

            return(d);
        }

};

std::string pathFind(int xStart, int yStart, int xFinish, int yFinish, myMap* map);
void VisualizePath(std::string route, myMap *map, ros::Publisher pub);
std::vector <geometry_msgs::Point> string2Vectors(std::string path, myMap *map);
std::vector <geometry_msgs::Point> PathShortener(std::vector <geometry_msgs::Point> path, sensor_msgs::PointCloud* obstacles, float robotWidth);
#endif // NODE_H
