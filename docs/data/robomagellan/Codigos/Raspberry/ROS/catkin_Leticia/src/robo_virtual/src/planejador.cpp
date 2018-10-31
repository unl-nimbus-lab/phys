#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"

#include <sstream>
#include <vector>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "planejador");

  ros::NodeHandle n;

  ros::Publisher trajetoriaPub = n.advertise<nav_msgs::Path>("path_planned", 1000);
  ros::Publisher pubEnable = n.advertise<std_msgs::Int16>("enable_follow_path", 1000);

  sleep(1);

  nav_msgs::Path trajeto;
  std::vector<geometry_msgs::PoseStamped> pose;
  geometry_msgs::PoseStamped auxPosition;

  float contador = 0;

  ros::Rate loop_rate(1);

  /*while (contador < 8) {
    auxPosition.header.seq = (int)contador;
    auxPosition.pose.position.x = contador;
    auxPosition.pose.position.y = contador;
    auxPosition.pose.position.z = contador;
    trajeto.poses.push_back(auxPosition);
    contador += 1;
  }*/
    /*auxPosition.header.seq = (int)contador;
    auxPosition.pose.position.x = 0;
    auxPosition.pose.position.y = 0;
    auxPosition.pose.position.z = contador;
    trajeto.poses.push_back(auxPosition);*/
   /* auxPosition.header.seq = (int)contador;
    auxPosition.pose.position.x = 0;
    auxPosition.pose.position.y = 0;
    auxPosition.pose.position.z = contador;
    trajeto.poses.push_back(auxPosition);

    auxPosition.header.seq = (int)contador;
    auxPosition.pose.position.x = 0;
    auxPosition.pose.position.y = 0;
    auxPosition.pose.position.z = contador;
    trajeto.poses.push_back(auxPosition);

    auxPosition.header.seq = (int)contador;
    auxPosition.pose.position.x = 9;
    auxPosition.pose.position.y = 0;
    auxPosition.pose.position.z = contador;
    trajeto.poses.push_back(auxPosition);

    auxPosition.header.seq = (int)contador;
    auxPosition.pose.position.x = 0;
    auxPosition.pose.position.y = 0;
    auxPosition.pose.position.z = contador;
    trajeto.poses.push_back(auxPosition);

    auxPosition.header.seq = (int)contador;
    auxPosition.pose.position.x = 0;
    auxPosition.pose.position.y = -9;
    auxPosition.pose.position.z = contador;
    trajeto.poses.push_back(auxPosition);

    auxPosition.header.seq = (int)contador;
    auxPosition.pose.position.x = 0;
    auxPosition.pose.position.y = 0;
    auxPosition.pose.position.z = contador;
    trajeto.poses.push_back(auxPosition);

    auxPosition.header.seq = (int)contador;
    auxPosition.pose.position.x = 0;
    auxPosition.pose.position.y = 9;
    auxPosition.pose.position.z = contador;
    trajeto.poses.push_back(auxPosition);

    auxPosition.header.seq = (int)contador;
    auxPosition.pose.position.x = 9;
    auxPosition.pose.position.y = 9;
    auxPosition.pose.position.z = contador;
    trajeto.poses.push_back(auxPosition);

    auxPosition.header.seq = (int)contador;
    auxPosition.pose.position.x = 0;
    auxPosition.pose.position.y = 0;
    auxPosition.pose.position.z = contador;
    trajeto.poses.push_back(auxPosition);
*/
    auxPosition.header.seq = (int)contador;
    auxPosition.pose.position.x = 9;
    auxPosition.pose.position.y = 9;
    auxPosition.pose.position.z = contador;
    trajeto.poses.push_back(auxPosition);

    auxPosition.header.seq = (int)contador;
    auxPosition.pose.position.x = 0;
    auxPosition.pose.position.y = 0;
    auxPosition.pose.position.z = contador;
    auxPosition.pose.orientation.w = 0.707;
    auxPosition.pose.orientation.y = 0;
    auxPosition.pose.orientation.x = 0;
    auxPosition.pose.orientation.z = -0.707;
    trajeto.poses.push_back(auxPosition);


/*
    auxPosition.header.seq = (int)contador;
    auxPosition.pose.position.x = 0;
    auxPosition.pose.position.y = 0;
    auxPosition.pose.position.z = contador;
    trajeto.poses.push_back(auxPosition);

    auxPosition.header.seq = (int)contador;
    auxPosition.pose.position.x = 0;
    auxPosition.pose.position.y = 9;
    auxPosition.pose.position.z = contador;
    trajeto.poses.push_back(auxPosition);

    auxPosition.header.seq = (int)contador;
    auxPosition.pose.position.x = 0;
    auxPosition.pose.position.y = 0;
    auxPosition.pose.position.z = contador;
    trajeto.poses.push_back(auxPosition);

    auxPosition.header.seq = (int)contador;
    auxPosition.pose.position.x = 9;
    auxPosition.pose.position.y = 0;
    auxPosition.pose.position.z = contador;
    trajeto.poses.push_back(auxPosition);

    auxPosition.header.seq = (int)contador;
    auxPosition.pose.position.x = 0;
    auxPosition.pose.position.y = 0;
    auxPosition.pose.position.z = contador;
    trajeto.poses.push_back(auxPosition);

    auxPosition.header.seq = (int)contador;
    auxPosition.pose.position.x = 0;
    auxPosition.pose.position.y = 0;
    auxPosition.pose.position.z = contador;
    trajeto.poses.push_back(auxPosition);
  */  /*auxPosition.header.seq = (int)contador;
    auxPosition.pose.position.x = 0;
    auxPosition.pose.position.y = 9;
    auxPosition.pose.position.z = contador;
    trajeto.poses.push_back(auxPosition);


    auxPosition.header.seq = (int)contador;
    auxPosition.pose.position.x = 0;
    auxPosition.pose.position.y = 0;
    auxPosition.pose.position.z = contador;
    trajeto.poses.push_back(auxPosition);

    auxPosition.header.seq = (int)contador;
    auxPosition.pose.position.x = 0;
    auxPosition.pose.position.y = 9;
    auxPosition.pose.position.z = contador;
    trajeto.poses.push_back(auxPosition);

    auxPosition.header.seq = (int)contador;
    auxPosition.pose.position.x = 9;
    auxPosition.pose.position.y = 9;
    auxPosition.pose.position.z = contador;
    trajeto.poses.push_back(auxPosition);

    auxPosition.header.seq = (int)contador;
    auxPosition.pose.position.x = 0;
    auxPosition.pose.position.y = 0;
    auxPosition.pose.position.z = contador;
    trajeto.poses.push_back(auxPosition);
*/
 /*   auxPosition.header.seq = (int)contador;
    auxPosition.pose.position.x = 0;
    auxPosition.pose.position.y = 9;
    auxPosition.pose.position.z = contador;
    trajeto.poses.push_back(auxPosition);
  while (contador > 0) {
    auxPosition.header.seq = (int)contador;
    auxPosition.pose.position.x = contador;
    auxPosition.pose.position.y = contador;
    auxPosition.pose.position.z = contador;
    trajeto.poses.push_back(auxPosition);
    contador -= 1;
  }*/
  
  std_msgs::Int16 enable;

  enable.data  = 1;

  pubEnable.publish(enable);
  trajetoriaPub.publish(trajeto);
  
  while (ros::ok())
  {
    /*static int count = 0;

    if (count == 10){
      trajeto.poses.clear();
      auxPosition.header.seq = (int)contador;
      auxPosition.pose.position.x = -2;
      auxPosition.pose.position.y = -2;
      auxPosition.pose.position.z = 0;
      trajeto.poses.push_back(auxPosition);
      trajetoriaPub.publish(trajeto);
      //enable.data = false;
      //pubEnable.publish(enable);
      ROS_INFO("NOVO TRAJETO");
    }
    count ++;*/
    ROS_INFO("Enviando trajeto ");
    
    ros::spinOnce();

    loop_rate.sleep();

  }

  return 0;
}
