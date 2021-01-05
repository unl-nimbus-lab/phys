/*
 * qlearner.h
 *
 *  Created on: Apr 2, 2011
 *      Author: titus
 */

#ifndef QLEARNER_H_
#define QLEARNER_H_

#include "ros/ros.h"
#include <XmlRpcValue.h>
#include <XmlRpcException.h>
#include <vector>
#include <cstdlib>
#include <time.h>

class QLearner
{
public:
  QLearner(ros::NodeHandle nh);
	void Update(double reward, int state, int state_p, int action);
	int GetAction(int state);
	void DecreaseTemp(void);
	std::string PrintTable(void);
	double QVal(int state, int action);
	
private:
  ros::NodeHandle n_;
	int num_actions_, num_states_, size_array_, temp_cnt_;
	double learning_rate_, discount_factor_, temp_const_, temp_alpha_, temp_;
	std::vector<double> q_array_;
	bool learn_;

	void Init(void);
	int GetMaxAction(int state);
	double GetMaxActionQVal(int state);
};

#endif /* QLEARNER_H_ */
