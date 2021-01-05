#ifndef BATTERYMODEL_H
#define BATTERYMODEL_H
#include <ros/ros.h>
#include <std_msgs/Float64.h>

//percent
#define MINIMUM_RECHARGE_LEVEL 60.0
//seconds
#define MAX_BATTERY_TIME 30*60.0

class BatteryModel
{
public:
    BatteryModel(ros::NodeHandle &n, std::string uavID);
    void setBatterytime(ros::Duration battery_time);
    float update(bool scientific_node_enable,bool motors_on);
    float getCurrentLevel();
    void mainLoop(const ros::TimerEvent& te);
    void setBatteryLevel(const std_msgs::Float64 new_level);

private:
    ros::Duration battery_time_;
    bool scientific_node_on_;
    bool motor_on_;
    ros::Timer main_loop_timer_;
    ros::Subscriber batLevelSub_;
};

#endif // BATTERYMODEL_H
