#include <batterymodel.h>

BatteryModel::BatteryModel(ros::NodeHandle &n, std::string uavID)
{
   main_loop_timer_ = n.createTimer(ros::Duration(1),
                                    boost::bind(
                                       &BatteryModel::mainLoop, this, _1));

   batLevelSub_ = n.subscribe(uavID.append("/setBatteryLevel"),
                              0,&BatteryModel::setBatteryLevel,this);
}
void BatteryModel::setBatterytime(ros::Duration battery_time)
{
   this->battery_time_ = battery_time;
}

float BatteryModel::update(bool scientific_node_enable,bool motors_on)
{
   if(scientific_node_enable && !scientific_node_on_)
   {
      //Hay que quitar 5 minutos de bateria.
      if(battery_time_.toSec()> (5*60))
         battery_time_ -= ros::Duration(5*60);
   }
   scientific_node_on_ = scientific_node_enable;
   this->motor_on_ = motors_on;

   return (battery_time_.toSec()/(MAX_BATTERY_TIME))*100.0;
}
float BatteryModel::getCurrentLevel()
{
   return (battery_time_.toSec()/(MAX_BATTERY_TIME))*100.0;
}

void BatteryModel::mainLoop(const ros::TimerEvent& te)
{
   if(motor_on_)
   {
      if(battery_time_.toSec()>1)
         battery_time_ -= ros::Duration(1);
   }
}

void BatteryModel::setBatteryLevel(const std_msgs::Float64 new_level) {
   //std::cerr << "BatteryModel::setBatteryLevel: "
   //<< new_level.data << std::endl;
   this->battery_time_ =
         ros::Duration((new_level.data/100.0)*MAX_BATTERY_TIME);
}
