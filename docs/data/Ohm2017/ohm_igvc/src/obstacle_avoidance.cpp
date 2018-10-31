#include "ros/ros.h"
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <math.h>  
#include "lidarpoint.h"
#include "obstacle.h"
using namespace std;

//Global variables
ros::Publisher autoControlLogicPublisher;
// ros::Publisher globalMappingPublisher = n.advertise<geometry_msgs::Twist>("obstacles", 1000);
vector<float> lidarData;
double theta;
vector<LidarPoint> lidarpoints;
vector<Obstacle> obstacles;
static double maxRadius = 10.0;
static double MM2M  = 0.001;
static int M2MM = 1000;
static double nonSeparationThresh = 200;
static int highThresh = 50;
static int lowThresh = 4;
int forgiveCount = 3;
int linkedCount;
int sumOfPoints;
int obsSizeNum;
bool isAlreadyLinking = false;
bool isThereAnObstacle = false;
double obstacleStopThreshold;
geometry_msgs::Pose2D robotDetectedDangerPosition;
geometry_msgs::Pose2D robotCurrentPosition;
double backupDistance;

//Computing Functions
void clearState()
{
    sumOfPoints = 0;
    linkedCount = 0;
    isThereAnObstacle = false;
}

void convertAllPointsToCartesian()
{
    double x = 0;
    double y = 0;
    double distanceFromRobot = 0;

    for(int i = 0; i < lidarData.size(); i++ )
    {
        x = cos(theta) * lidarData[i];
        y = sin(theta) * lidarData[i];
        distanceFromRobot = lidarData[i];

        LidarPoint newPoint(x, y);
        lidarpoints.push_back(newPoint);
        x = 0;
        y = 0;
    }
}
double distanceCalculator(LidarPoint lidarPoint1, LidarPoint lidarPoint2)
{
    return sqrt(pow((lidarPoint2.x - lidarPoint1.x), 2)- pow((lidarPoint2.y - lidarPoint1.y), 2));
}
void linkPoint(double currPointDist, double twoPointsDist)
{
    linkedCount += 1;
    sumOfPoints += currPointDist;
    obsSizeNum += twoPointsDist;
}
void addAndAnalyzeObstacle(int value, Obstacle& obstacle)
{
    double index = (value - linkedCount) / 2;
    double mag = sumOfPoints / linkedCount;
    // bool isOutsideTheField = false;

    obstacle.x = mag * cos(theta);
    obstacle.y = mag * sin(theta);

    if (mag < maxRadius || linkedCount > highThresh || linkedCount < lowThresh)
    {
        clearState();
    }
    else
    {
        // if(obstacle.x > 4.75 || obstacle.x < -1.750 || obstacle.y > 11.75 || obstacle.y < -2.75)// Needs to be rechecked
        // {
        //     isOutsideTheField = true;
        // }
        // if(!isOutsideTheField)
        // {
            if(mag <= obstacleStopThreshold) //if there is a close obstacle, in meters
            {
                isThereAnObstacle = true;
                ROS_INFO("Obstacle detected");
            }
            else
            {
                clearState();
            }
        // }
    }
}

void obstacleDetection()
{
    int j;
    for(int i = 360; i < lidarpoints.size() - 361; i++)
    {
        auto currentPoint = lidarpoints[i];
        bool isPointLinked = false;
        Obstacle obstacle;

        if(currentPoint.distanceFromRobot < maxRadius)
        {
            for(j = 1; j <= forgiveCount; j++ )
            {
                auto nextPoint = lidarpoints[i + 1];
                double pointsDistance = distanceCalculator(currentPoint, nextPoint);
                if(pointsDistance < nonSeparationThresh * j * MM2M)
                {
                    linkPoint(currentPoint.distanceFromRobot, pointsDistance);
                    isPointLinked = true;
                    if(!isAlreadyLinking)
                    { 
                        obstacle.objStartIndex = i;
                        isAlreadyLinking = true;
                    }
                    break;
                }
            }
        }
        if(isPointLinked == false)
        {
            if(isAlreadyLinking)
            {
                obstacle.objEndIndex = i;
                addAndAnalyzeObstacle(i, obstacle);
            }
            isAlreadyLinking = false;
            clearState();
        }
        else
        {
            i = i + j - 1;
            if(i > lidarpoints.size() - 361)
            {
                addAndAnalyzeObstacle(i, obstacle);
                clearState();
            }
        }
        
    }
}

//Callback Functions
void lidarPointsCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    lidarData = msg->ranges;
    theta = msg->angle_increment;

    convertAllPointsToCartesian();
    obstacleDetection();
}

void odometryCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    robotCurrentPosition = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_avoidance");
    // bool keepMoving;
    
    ros::NodeHandle n;
    n.param("obstacle_avoidance_obstacle_stop_threshold", obstacleStopThreshold, 3.0);
    n.param("obstacle_avoidance_backup_distance", backupDistance, 1.0);
    autoControlLogicPublisher = n.advertise<geometry_msgs::Twist>("avoidanceOverride", 1000);
    ros::Subscriber lidarSub = n.subscribe("scan", 3, lidarPointsCallback);
    ros::Subscriber odometrySub = n.subscribe("/ohm/odom", 3, odometryCallback);

    while(ros::ok()){
        if(isThereAnObstacle){
            double deltaDistance = sqrt(pow((robotCurrentPosition.x - robotDetectedDangerPosition.x), 2)- pow((robotCurrentPosition.y - robotDetectedDangerPosition.y), 2));
            while(ros::ok() && deltaDistance < backupDistance){
                geometry_msgs::Twist msg;

                msg.linear.x = -0.5;
                msg.angular.y = 0;

                autoControlLogicPublisher.publish(msg);
            }
        }
        ros::spin_once();
    }

    return 0;
}