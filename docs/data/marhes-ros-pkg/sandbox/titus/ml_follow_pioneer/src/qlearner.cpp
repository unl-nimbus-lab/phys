#include "ml_light_pioneer/qlearner.h"

QLearner::QLearner(ros::NodeHandle nh)
{
  n_ = nh;
  ros::NodeHandle n_private("~");
  n_private.param("num_states", num_states_, 8);
  n_private.param("num_actions", num_actions_, 3);
  n_private.param("learning_rate", learning_rate_, 0.3);
  n_private.param("discount_factor", discount_factor_, 0.5);
  n_private.param("temp_const", temp_const_, 5.0);
  n_private.param("temp_alpha", temp_alpha_, 0.85);
  
  temp_cnt_ = 0;
  temp_ = temp_const_ * pow(temp_alpha_, temp_cnt_);
  size_array_ = num_actions_ * num_states_;

  srand ( time(NULL) );
 	Init();
 	
  if (n_private.hasParam("qarray"))
  {
    learn_ = false;
    
    // Get the qtable
    XmlRpc::XmlRpcValue qtable;
	  n_private.getParam("qarray", qtable);
	  if (qtable.getType() != XmlRpc::XmlRpcValue::TypeArray)
		  ROS_ERROR("Error reading footsteps/x from config file.");
		  
    int size;
		try
		{
			size = qtable.size();
			
			if (size != (num_states_ * num_actions_))
			{
				ROS_ERROR("Size of array does not match num_states * num_actions = %d,\
				           exiting.", num_states_ * num_actions_);
				exit(0);
			}
		} 
		catch (const XmlRpc::XmlRpcException e)
		{
			ROS_ERROR("No table available, exiting.");
			exit(0);
		}

		// create qarray set
		for(int i=0; i < size; i++)
		{
			q_array_.push_back((double)qtable[i]);
		}
  }
  else
  {
    learn_ = true;
    double floor = -0.1, ceiling = 0.1, range = (ceiling - floor);
    q_array_ = std::vector<double>(size_array_);
    for (int i = 0; i < size_array_; i++)
    {
      q_array_[i] = floor + (range * rand()) / (RAND_MAX + 1.0);
    }
  } 
  
  //ROS_INFO("%s", PrintTable().c_str()); 
}

void QLearner::Update(double reward, int state, int state_p, int action)
{
	if (state >= num_states_ || state < 0 ||
		state_p >= num_states_ || state_p < 0 ||
		action >= num_actions_ || action < 0)
	{
		ROS_FATAL("Update args out of bounds.");
	}

	// State then action
	int vector_index = state * num_actions_ + action;
	double max_future_val = GetMaxActionQVal(state_p);
	q_array_[vector_index] += learning_rate_ * (reward + discount_factor_ *
			max_future_val - QVal(state, action));
}

int QLearner::GetAction(int state)
{
	int action, i;
  double probs[num_actions_], sum = 0, r;
  
	if (learn_)
	{
  	for (i = 0; i < num_actions_; i++)
	  {
	    probs[i] = exp(QVal(state, i) / temp_);
	    sum += probs[i];
	  }
	
	  for (i = 0; i < num_actions_; i++)
	  {
	    probs[i] = probs[i] / sum;
	  }
    
    r = rand() / (RAND_MAX + 1.0);
    i = 0;
    sum = 0;
    while (i < num_actions_)
    {
      sum += probs[i];
      if (r < sum)
        break;
      else
        i++;
    }
    
    action = i;
  }
  else
  {
    action = GetMaxAction(state);
  }
  
  return action;
}

void QLearner::Init(void)
{
	if (learning_rate_ > 1 || learning_rate_ <= 0)
		ROS_FATAL("Learning rate is not 0 < lr <= 1.");

	if (discount_factor_ >= 1 || discount_factor_ < 0)
		ROS_FATAL("Discount factor is not 0 <= df < 1.");
}

int QLearner::GetMaxAction(int state)
{
  int maxAction = 0;
  double maxQVal = QVal(state, 0);
  
  for (int i = 1; i < num_actions_; i++)
  {
    if (maxQVal < QVal(state, i))
    {
      maxQVal = QVal(state, i);
      maxAction = i;
    }
  }
  return maxAction;
}

double QLearner::GetMaxActionQVal(int state)
{
  double maxQVal = QVal(state, 0);
  
  for (int i = 1; i < num_actions_; i++)
  {
    if (maxQVal < QVal(state, i))
    {
      maxQVal = QVal(state, i);
    }
  }
  return maxQVal;
}

double QLearner::QVal(int state, int action)
{
  int vector_index = state * num_actions_ + action;
  return q_array_[vector_index];
}

void QLearner::DecreaseTemp(void)
{
  temp_cnt_++;
  temp_ = temp_const_ * pow(temp_alpha_, temp_cnt_);
  if (temp_ < 0.05)
    temp_ = 0.05;
}

std::string QLearner::PrintTable(void)
{
	std::string s;
	for(int state = 0; state < num_states_; state++)
	{
		for(int action = 0; action < num_actions_; action++)
		{
			s.append(boost::lexical_cast<std::string>(q_array_[state * num_actions_ + action]) + ", ");
		}
		s.append("\n");
	}

	return s;
}

/*
int main(int argc, char **argv)
{
	ros::init(argc, argv, "qtable_test");
	ros::NodeHandle n;

  QLearner* s = new QLearner(n);
  ros::spin();

  return 0;
}
*/
