#ifndef EYE_CONTACT_H_
#define EYE_CONTACT_H_

#include <vector>
#include <ros/ros.h>
#include <iostream>
#include <tf/tf.h>
#include <tf/transform_listener.h>
//#include <tf/LinearMath/Quaternion.h>
#include "humans_msgs/Humans.h"
#include "eyeballs_msgs/Eyeballs.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/thread.hpp>

#include <fstream>
#include <functional>
#include <algorithm>
#include <numeric>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace eye_contact {

  class EyeContact {
  public:
    EyeContact();

    virtual ~EyeContact();

    void Callback(const humans_msgs::HumansConstPtr& msg);

    void micro_motion_switch(int now_micro_motion);

    int blink_check(std::vector<humans_msgs::DegConf> open_level);

    bool contact_check(humans_msgs::Direction dir, humans_msgs::XYConf gaze_dir);

    bool contact_check_noface(humans_msgs::Direction dir, humans_msgs::XYConf gaze_dir);

    int face_right_and_left_check(int right_or_left);

    void eyecontact_check(int now_state);

    void name_and_id_check(std::vector<humans_msgs::Person> persons);

    void face_not_found_case(int now_state);

    int GetRandom(int min, int max);

    int PointEyeContactDirAndGaze(int f_horizon, int g_horizon, int f_conf, int g_conf);

    int EyeContactDirAndGaze(int f_horizon, int g_horizon, int f_conf, int g_conf);

    void moveThread();

    void GetRPY(const geometry_msgs::Quaternion &q,double &roll,double &pitch,double &yaw);
    void GetTfRPY(const tf::Quaternion btq, double &roll,double &pitch,double &yaw);

    void GetQtFromYaw(const geometry_msgs::Quaternion &quat, double yaw);

    bool qtRoughlyEq(geometry_msgs::Quaternion src1, geometry_msgs::Quaternion src2);

    void publishZeroVelocity();


  private:

    ros::NodeHandle nh;
    ros::Subscriber eye_sub;
    ros::Publisher eye_pub, vel_pub;
    eyeballs_msgs::Eyeballs ebs;
    nav_msgs::OdometryConstPtr now_odom;
    geometry_msgs::PoseWithCovarianceStampedConstPtr now_pose;
    boost::thread* move_thread;
    
    tf::TransformListener listener;

    std::vector<int> point_buff;
    int queue_size;
    int tolerance;
    int conf_th;
    int contact_count;
    int img_width;
    int img_height;

    int dir_horizon;
    int gaze_horizon;
    int dir_conf;
    int gaze_conf;

    bool contact_state;
    int gap;
    
    double count_time[4];
    double threshold_time[4];
    double tm_param[4];
    double test,fps;
    
    //stdstringstream file_name;
    int state;
    bool move_state;
    double former_time, now_time;
    
    int micro_motion;
    int look_motion;
    int blink_torf;
    int not_found;
  };

};

#endif
