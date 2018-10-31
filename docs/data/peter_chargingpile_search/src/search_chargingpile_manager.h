#pragma once


#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <geometry_msgs/Twist.h>

#include "global_data_structure.h"



namespace peter_chargingpile_search {


class SearchChargingPileFSM;


class SearchChargingPileManager
{
public:
    SearchChargingPileManager(ros::NodeHandle);

    void addLaserScanMsg(const sensor_msgs::LaserScanConstPtr& msg);

    double _param_break_distance;
    int _param_ignore_point_num;
    double _param_max_salient_tolerance;
    double _param_max_variance_tolerance;
    double _param_max_festureangle_tolerance;
    double _param_angular_vel;
    double _param_linear_vel;

private:
    boost::shared_ptr<SearchChargingPileFSM> _searchFSM;

    std::vector<PointInfo> _sourcePoints;

    std::vector<double> _breakedLaserRange;
    std::vector<double> _breakedLaserAngle;

    std::vector<double> _filterLaserRange;
    std::vector<double> _filterLaserAngle;

    std::vector<std::vector<XYPoint> > _splitLines;

    static const int Const_Queue_Length;
    static const double Recognization_Angle;
    static const double LaserXOffsetToRobot;

    static const double Break_Distance;
    static const int    Ignore_Point_Num;
    static const double Max_Salient_Tolerance;
    static const double Max_Variance_Tolerance;
    static const double Max_FestureAngle_Tolerance;


private:

    //mathematics
    int  splitLaserWithRange();

    void filterSplitLaser(std::vector<double> angles, std::vector<double> ranges);

    void changeRangetoXY(std::vector<double> angles, std::vector<double> ranges);

    // ================method 1================
    int  findSalientIndexByVector(std::vector<XYPoint> line, double eps);
    bool findKeyPointFromLines(PointWithTimeStamp& keyPoint);

    // ================method 2================
    void recurSplitPointsVector(std::vector<XYPoint> line, double eps, std::vector<weighted_fit::LinePara>& lineParas);
    bool recurFindKeyPointFromLines(PointWithTimeStamp& keyPoint);

private:
    ros::NodeHandle _nh;
    ros::Publisher _pointsPub;
    void publishPointCloud(std::vector<double> angles, std::vector<double> ranges,
                           PointWithTimeStamp keyPoint, bool isShowKeyPoint);

    ros::Publisher _ctrlCmdVelPub;
    ros::Timer     _ctrlCmdVelTimer;
    void onTimerCtrlCmdVel(const ros::TimerEvent& t);
    geometry_msgs::Twist _vel_msg;


private:
    bool _chargeOrderFlag;
    bool _powerStatusFlag;



};


}
