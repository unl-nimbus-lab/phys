
#include "robot_state.h"
#include <ros/ros.h>


namespace peter_chargingpile_search {


// ====================================

InitialState::InitialState(SearchChargingPileFSM* fsm)
{
    _fsm = fsm;
}


void InitialState::update(UpdateDataPacket& pack, tf::Transform& transform)
{
    if (pack.chargeOrderFlag)
    {
        _fsm->changeState(_fsm->_scanLandmarkState);
    }
}




// =============================

ScanLandmarkState::ScanLandmarkState(SearchChargingPileFSM* fsm)
{
    _fsm = fsm;
}


void ScanLandmarkState::update(UpdateDataPacket& pack, tf::Transform& transform)
{

    if (ros::Time::now() - pack.objPosition.timestamp < ros::Duration(0.5))
    {
        _fsm->changeState(_fsm->_approachingState);
    }
}

// ==============================

const double ApproachingState::XOffsetToObjective = 0.5;              //unit:m

ApproachingState::ApproachingState(SearchChargingPileFSM* fsm)
{
    _fsm = fsm;
//    _isFoundObjectivePoint = false;
}

void ApproachingState::update(UpdateDataPacket& pack, tf::Transform& transform)
{

//        ROS_INFO_STREAM("ApproachingState- " << pack.objPosition.timestamp << "-->(x, y, z, theta) = " <<
//                        "(" <<
//                        pack.objPosition.x << " " <<
//                        pack.objPosition.y << " " <<
//                        pack.objPosition.z << " " <<
//                        pack.objPosition.theta << " " <<
//                        ")");


    //xiwrong-->todo    timestamp 3s

//    if (!_isFoundObjectivePoint)
//    {
//        _isFoundObjectivePoint  = true;
//        _oldPoint = pack.objPosition;
//    }

    double deltaX = pack.objPosition.x - XOffsetToObjective* cos(pack.objPosition.theta);
    double deltaY = pack.objPosition.y - XOffsetToObjective* sin(pack.objPosition.theta);
    double deltaTheta = pack.objPosition.theta;

    ROS_INFO_STREAM("ApproachingState- deltaX Y Theta = " << deltaX << " " << deltaY << " " <<deltaTheta);

    if (deltaX < 0.02 && deltaY < 0.005 /*&& deltaTheta < 0.1*/)
    {
        _fsm->changeState(_fsm->_secondApproachingState);
    }

    transform.setOrigin(tf::Vector3(deltaX, deltaY, pack.objPosition.z));
    tf::Quaternion q;
    q.setRPY(0, 0, deltaTheta);
    transform.setRotation(q);


}

//double ApproachingState::distanceBetweenTwoPoints(PointWithTimeStamp a, PointWithTimeStamp b)
//{
//    return sqrt(pow(a.x - b.x, 2.0) + pow(a.y - b.y, 2.0));
//}

// ================================

SecondApproachingState::SecondApproachingState(SearchChargingPileFSM* fsm)
{
    _fsm = fsm;
}

void SecondApproachingState::update(UpdateDataPacket& pack, tf::Transform& transform)
{
    ROS_INFO("SecondApproachingState- update");


    transform.setOrigin(tf::Vector3(pack.objPosition.x, pack.objPosition.y, pack.objPosition.z));
    tf::Quaternion q;
    q.setRPY(0, 0, pack.objPosition.theta);
    transform.setRotation(q);

    if (pack.powerStatusFlag)
    {
        _fsm->changeState(_fsm->_finishState);
    }
}


// ================================
FinishState::FinishState(SearchChargingPileFSM* fsm)
{
    _fsm = fsm;
}

void FinishState::update(UpdateDataPacket& pack, tf::Transform& transform)
{
    //    ROS_INFO("FinishState...");
}

// ===============================



}
