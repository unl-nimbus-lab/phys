#pragma once


#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>


#include <tf/transform_broadcaster.h>


namespace peter_chargingpile_search {


class RobotState;
struct UpdateDataPacket;

class SearchChargingPileFSM
{
private:
    boost::shared_ptr<RobotState> _currentState;

public:
    boost::shared_ptr<RobotState> _initialState;
    boost::shared_ptr<RobotState> _scanLandmarkState;
    boost::shared_ptr<RobotState> _approachingState;
    boost::shared_ptr<RobotState> _secondApproachingState;
    boost::shared_ptr<RobotState> _finishState;

    SearchChargingPileFSM();
    ~SearchChargingPileFSM();

    void begin();
    void update(UpdateDataPacket& pack, tf::Transform& transform);
    void end();
    void changeState(boost::shared_ptr<RobotState> state);

private:

    tf::TransformBroadcaster _expectedPoseTFbr;
};

//xiwrong-->p
//typedef boost::shared_ptr<SearchChargingPileFSM> SearchFMSPtr;
//typedef boost::shared_ptr<SearchChargingPileFSM const> ConstSearchFMSPtr;
}
