/**
 *  This header file defines the Node class. It is highly recommended
 *  whenever an oriented-object programming ROS Node class is created
 *  to enhance this one.
 *
 *  Version: 1.0.0
 *  Created on: 05/10/2016
 *  Modified on: 21/10/2016
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@gmail.com)
 *  Maintainer: Professores de ECA419 (eca419.unifei@gmail.com)
 */

#ifndef _NODE_H_
#define _NODE_H_

#include <ros/ros.h>

class Node
{
public:
  virtual ~Node(); // destructor
  virtual void spin(); // standard spin method (according to the given loop rate)

protected:
  Node(ros::NodeHandle *nh, float loop_rate); // protected constructor
  ros::NodeHandle* getNodeHandle() const;
  std::string getName() const;
  void shutdown() const;
  
private:
  float loop_rate_; // positive spin rate
  std::string name_; // ROS node name
  ros::NodeHandle *nh_; // private ros node handle (has-a relationship)
  virtual void controlLoop() = 0;
};

#endif // _NODE_H_
