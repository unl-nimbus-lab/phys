/*************************************************************************
 *
 * FADA-CATEC
 * __________________
 *
 *  [2013] FADA-CATEC
 *  All Rights Reserved.
 *
 * NOTICE:  All information contained herein is, and remains
 * the property of FADA-CATEC and its suppliers,
 * if any.  The intellectual and technical concepts contained
 * herein are proprietary to FADA-CATEC
 * and its suppliers and may be covered by Europe and Foreign Patents,
 * patents in process, and are protected by trade secret or copyright law.
 * Dissemination of this information or reproduction of this material
 * is strictly forbidden unless prior written permission is obtained
 * from FADA-CATEC.
 *
 * Created on: 23-Oct-2012
 * Engineer: Jonathan Ruiz Páez
 * Email: jruiz@catec.aero
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include "common/PID.hh"
#include <catec_msgs/PositionControlParameters.h>

#define LAND_Z 0.20
#define TAKE_OFF_Z 0.3

using namespace std;
using namespace geometry_msgs;
using namespace gazebo;



extern LandActionClass *land_action;
extern TakeOffActionClass *take_off_action;

ros::Subscriber sub_stage, pid_parameters;
ros::Publisher stagevel_pub, state_pub;

common::PID pid_x,pid_y,pid_z,pid_yaw;
double pid_x_windup,pid_y_windup,pid_z_windup;
double pid_x_sat, pid_y_sat, pid_z_sat;
ros::Time lastTime;


void CommandFlagsCallback(const uint land_takeoff)
{
  if(land_takeoff==1) //LAND
  {
    if(actual_state != LANDED && actual_state != LANDING)
    {
      //ROS_INFO("CmdFlag receive, landing...");
      actual_state = LANDING;
      cmdFlagTime =  ros::Time::now();
      cmdFlagDuration.fromSec(3);


      lastControlReferenceRw.c_reference_rw.position.x = lastPose.position.x;
      lastControlReferenceRw.c_reference_rw.position.y = lastPose.position.y;
      lastControlReferenceRw.c_reference_rw.position.z = LAND_Z;





    }
  }
  else if(land_takeoff==2)//TAKE_OFF
  {
    if(actual_state != FLYING  && actual_state != TAKING_OFF && actual_state != LANDING)
    {
      //ROS_INFO("CmdFlag receive, taking off...");
      actual_state = TAKING_OFF;
      cmdFlagTime =  ros::Time::now();
      cmdFlagDuration.fromSec(3);


      lastControlReferenceRw.c_reference_rw.position.x = lastPose.position.x;
      lastControlReferenceRw.c_reference_rw.position.y = lastPose.position.y;
      lastControlReferenceRw.c_reference_rw.position.z = lastPose.position.z + TAKE_OFF_Z;

    }
  }
return;
}

Twist Update(Twist *res)
{

  if(actual_state == TAKING_OFF && (ros::Time::now() - cmdFlagTime >= cmdFlagDuration) && lastPose.position.z >= TAKE_OFF_Z-0.05)
  {
    //ROS_INFO("State change to Taked off.");
    actual_state = FLYING;
  }else if(actual_state == LANDING && (ros::Time::now() - cmdFlagTime >= cmdFlagDuration) && lastPose.position.z <= LAND_Z+0.05)
  {
    //ROS_INFO("State change to Landed.");
    actual_state = LANDED;
  }


  //Twist res;

  res->linear.x = 0;
  res->linear.y = 0;
  res->linear.z = -0.15;

  res->angular.x = 0;
  res->angular.y = 0;
  res->angular.z = 0;

  //Set maxvelocity into pid
  if(lastControlReferenceRw.c_reference_rw.cruise < 2.0)
  {
    pid_x.SetCmd(lastControlReferenceRw.c_reference_rw.cruise);
    pid_y.SetCmd(lastControlReferenceRw.c_reference_rw.cruise);
    pid_z.SetCmd(lastControlReferenceRw.c_reference_rw.cruise);
  }

//Temporal mientras se tunean los PIDS.
  if(actual_state!=LANDED)
  {


    Pose last_pose_cmd_;
    last_pose_cmd_.position.x =
        lastControlReferenceRw.c_reference_rw.position.x;
    last_pose_cmd_.position.y =
        lastControlReferenceRw.c_reference_rw.position.y;

    //Tenemos que rotar las posiciones.

    if(actual_state!=LANDING &&
        lastControlReferenceRw.c_reference_rw.position.z < TAKE_OFF_Z)
    {
      last_pose_cmd_.position.z = TAKE_OFF_Z;
    }
    else
    {
      last_pose_cmd_.position.z =
          lastControlReferenceRw.c_reference_rw.position.z;
    }

    while(lastControlReferenceRw.c_reference_rw.heading > M_PI)
        lastControlReferenceRw.c_reference_rw.heading -= M_PI;

      while(lastControlReferenceRw.c_reference_rw.heading < -M_PI)
        lastControlReferenceRw.c_reference_rw.heading += M_PI;

     // last_pose_cmd_.orientation = tf::createQuaternionMsgFromYaw(lastControlReferenceRw.c_reference_rw.heading);


    //Convert to vector 3
    tf::Vector3 tf_pos(lastPose.position.x,lastPose.position.y,lastPose.position.z);
    tf::Vector3 wanted_position(last_pose_cmd_.position.x,last_pose_cmd_.position.y,last_pose_cmd_.position.z);


    tf::Quaternion q_rot;
    q_rot.setX(lastPose.orientation.x);
    q_rot.setY(lastPose.orientation.y);
    q_rot.setZ(lastPose.orientation.z);
    q_rot.setW(lastPose.orientation.w);

    tf::Transform xform;
    xform.setOrigin(tf_pos);
    xform.setRotation(q_rot);


    tf::Vector3 rotated_position = xform.inverse() * wanted_position;




    /*cerr << "Punto       : [" << lastControlReferenceRw.c_reference_rw.position.x
       << ";" << lastControlReferenceRw.c_reference_rw.position.y
       << ";" << lastControlReferenceRw.c_reference_rw.position.z << "]" << endl;

    cerr << "Punto Rotado: [" << rotated_position.getX()
       << ";" << rotated_position.getY()
       << ";" << rotated_position.getZ() << "]" << endl;
*/

//		cerr << "Current State: " << actual_state << endl;



    ros::Duration rosTimeIncrement = ros::Time::now() - lastTime;

    common::Time timeIncrement(rosTimeIncrement.sec,rosTimeIncrement.nsec);

    double actualYaw = tf::getYaw(lastPose.orientation);


    //Antiwindup
    if(abs(res->linear.x) >= abs(pid_x_sat))
    {
      bool ok = (res->linear.x>0 && pid_x_sat>0) || (res->linear.x<0 && pid_x_sat<0);

      if(ok)
        pid_x.SetIGain(0);
      else
        pid_x.SetIGain(pid_x_windup);
    }
    else
      pid_x.SetIGain(pid_x_windup);


    if(abs(res->linear.y) >= abs(pid_y_sat))
    {
      bool ok = (res->linear.y>0 && pid_y_sat>0) || (res->linear.y<0 && pid_y_sat<0);

      if(ok)
        pid_y.SetIGain(0);
      else
        pid_y.SetIGain(pid_y_windup);
    }
    else
      pid_y.SetIGain(pid_y_windup);


    res->linear.x = pid_x.Update(rotated_position.getX()*(-1),
                   timeIncrement);
    res->linear.y = pid_y.Update(rotated_position.getY()*(-1),
                   timeIncrement);
    res->linear.z = pid_z.Update((last_pose_cmd_.position.z-lastPose.position.z)*(-1),
                   timeIncrement);
    res->angular.z = pid_yaw.Update((lastControlReferenceRw.c_reference_rw.heading-actualYaw)*(-1),timeIncrement);

  //	cerr << "Error x: "<< lastPose.position.x - last_pose_cmd_.position.x << endl;

    //cerr << "ActualYAW " << actualYaw << " referencia: " << lastControlReferenceRw.c_reference_rw.heading << endl;

    lastTime = ros::Time::now();

    /*	if(actual_state!=LANDING && lastPose.position.z < TAKE_OFF_Z && t.linear.z < 0)
      {
        t.linear.z = 0;
      }


      t.angular.z =(tf::getYaw(p.orientation) - tf::getYaw(actual.orientation));

      if(t.angular.z <= -PI)
      {
        t.angular.z += 2*PI;
      }
      if(t.angular.z >= PI)
      {
        t.angular.z -= 2*PI;
      }
      t.angular.x = t.angular.y  = 0;

      if(fabs(t.angular.z) > 0.5) {
        t.angular.z = t.angular.z*0.5/fabs(t.angular.z);
      }*/

      //ROS_INFO("Cmd Vel: x: %.3f  y: %.3f  z: %.3f", t.linear.x,t.linear.y,t.linear.z);
      //ROS_INFO("Actual : x: %.3f  y: %.3f  z: %.3f", lastVel.linear.x,lastVel.linear.y,lastVel.linear.z);
      //ROS_INFO("Cmd Vel: vmax: %f  amax: %f", vmax,amax);
    //	geometry_msgs::Vector3 vout = limitva(t.linear, lastVel.linear, 1/timerate, vmax, amax);

      //ROS_INFO("Limitad: x: %.3f  y: %.3f  z: %.3f", vout.x,vout.y,vout.z);


      //Pasamos a velocidad Relativa;


      //t.linear = vout;

    //	t.linear.z = vout.z;
      //vout.z = 0;
    //	t.linear.x = vout.x;
    //	t.linear.y = vout.y;

      /*tf::Vector3 salida;

      tf::vector3MsgToTF(vout,salida);

      angle += tf::getYaw(actual.orientation) + PI/2;
      if(angle <= -PI)
      {
        angle += 2*PI;
      }
      if(angle >= PI)
      {
        angle -= 2*PI;
      }*/



    //res->linear = t.linear;
    //res->angular.z =t.angular.z;

    }
    return *res;

}

void link_loop(const ros::TimerEvent& te)
{
  Twist res;

  Update(&res);

  land_action->uavUpdateState(actual_state);
  take_off_action->uavUpdateState(actual_state);

  /*res.linear.x = 0;
  res.linear.y = 0;
  res.linear.z = 0;
  res.angular.x = 0;
  res.angular.y = 0;
  res.angular.z = 0;*/

/*	Twist res;
  Update(&res);

  Vector3 vtemp;



  double rotation_angle =2*PI - tf::getYaw(lastPose.orientation);
  vtemp.x = cos(rotation_angle)*res.linear.x - sin(rotation_angle)*res.linear.y ;
  vtemp.y = cos(rotation_angle)*res.linear.y + sin(rotation_angle)*res.linear.x ;


  res.linear.x = vtemp.x;
  res.linear.y= vtemp.y;

  //Actualizamos el estado en las actions
  land_action->uavUpdateState(actual_state);
  take_off_action->uavUpdateState(actual_state);

  //Intercambiamos linear x por y, para que se corresponda con el testbed

  res.linear.x = res.linear.y;
  res.linear.y = res.linear.x*(-1);*/


  //Limit velocity to command.
  tf::Vector3 actualVel(res.linear.x, res.linear.y, res.linear.z);

  if(lastControlReferenceRw.c_reference_rw.cruise>0 && actualVel.length()>lastControlReferenceRw.c_reference_rw.cruise)
  {
    actualVel = (actualVel/actualVel.length())*lastControlReferenceRw.c_reference_rw.cruise;
  }

  res.linear.x = actualVel.x();
  res.linear.y = actualVel.y();
  res.linear.z = actualVel.z();
  stagevel_pub.publish(res);
}

void GazeboCallback(const nav_msgs::Odometry::ConstPtr& fp) {

  //Intercambiamos linear x por y, para que se corresponda con el testbed
  lastPose.position.x = fp->pose.pose.position.x;
  lastPose.position.y = fp->pose.pose.position.y;
  lastPose.position.z = fp->pose.pose.position.z;

  lastPose.orientation.x = fp->pose.pose.orientation.x;
  lastPose.orientation.y = fp->pose.pose.orientation.y;
  lastPose.orientation.z = fp->pose.pose.orientation.z;
  lastPose.orientation.w = fp->pose.pose.orientation.w;

  lastVel.angular.x = fp->twist.twist.angular.x;
  lastVel.angular.y = fp->twist.twist.angular.y;
  lastVel.angular.z = fp->twist.twist.angular.z;


  lastVel.linear.x = fp->twist.twist.linear.x;
  lastVel.linear.y = fp->twist.twist.linear.y;
  lastVel.linear.z = fp->twist.twist.linear.z;

  catec_msgs::UALStateStamped res;

  res.header.frame_id = uavID;
  res.header.stamp = ros::Time::now();

  res.ual_state.cpu_usage = 50;
  res.ual_state.memory_usage = 50;
  res.ual_state.remaining_battery = 50;

  res.ual_state.dynamic_state.position.valid = 1;
  res.ual_state.dynamic_state.position.x = lastPose.position.x;
  res.ual_state.dynamic_state.position.y = lastPose.position.y;
  res.ual_state.dynamic_state.position.z = lastPose.position.z;

  res.ual_state.dynamic_state.orientation.valid = 1;


  tf::Quaternion q(lastPose.orientation.x,lastPose.orientation.y,
           lastPose.orientation.z,lastPose.orientation.w);
  tf::Matrix3x3(q).getRPY(res.ual_state.dynamic_state.orientation.x,
              res.ual_state.dynamic_state.orientation.y,
              res.ual_state.dynamic_state.orientation.z);

  //res.ual_state.dynamic_state.orientation.x = lastPose.orientation.x;
  //res.ual_state.dynamic_state.orientation.y = lastPose.orientation.y;
  //res.ual_state.dynamic_state.orientation.z = lastPose.orientation.z;

  res.ual_state.dynamic_state.velocity.valid = 1;
  res.ual_state.dynamic_state.velocity.x = lastVel.linear.x;
  res.ual_state.dynamic_state.velocity.y = lastVel.linear.y;
  res.ual_state.dynamic_state.velocity.z = lastVel.linear.z;

  state_pub.publish(res);


  return ;
}
void PositionControlPIDCallback(const catec_msgs::PositionControlParameters::ConstPtr& fp) {
  switch(fp->pid)
  {
  case 0:

    pid_x.SetPGain(fp->Kp);
    pid_x.SetDGain(fp->Kd);
    pid_x.SetIGain(fp->Ki);
    pid_x.SetIMax(fp->i_max);
    pid_x.SetIMin(fp->i_min);
    pid_x.SetCmdMax(fp->max_velocity);
    pid_x.SetCmdMin(fp->max_velocity * (-1));
    pid_x_windup = fp->Ki;
    pid_x_sat = fp->max_velocity;
    break;
  case 1:
    pid_y.SetPGain(fp->Kp);
    pid_y.SetDGain(fp->Kd);
    pid_y.SetIGain(fp->Ki);
    pid_y.SetIMax(fp->i_max);
    pid_y.SetIMin(fp->i_min);
    pid_y.SetCmdMax(fp->max_velocity);
    pid_y.SetCmdMin(fp->max_velocity * (-1));
    pid_y_windup = fp->Ki;
    pid_y_sat = fp->max_velocity;
    break;
  case 2:
    pid_z.SetPGain(fp->Kp);
    pid_z.SetDGain(fp->Kd);
    pid_z.SetIGain(fp->Ki);
    pid_z.SetIMax(fp->i_max);
    pid_z.SetIMin(fp->i_min);
    pid_z.SetCmdMax(fp->max_velocity);
    pid_z.SetCmdMin(fp->max_velocity * (-1));
    pid_z_windup = fp->Ki;
    pid_z_sat = fp->max_velocity;
    break;
  case 3:
    pid_yaw.SetPGain(fp->Kp);
    pid_yaw.SetDGain(fp->Kd);
    pid_yaw.SetIGain(fp->Ki);
    pid_yaw.SetIMax(fp->i_max);
    pid_yaw.SetIMin(fp->i_min);
    pid_yaw.SetCmdMax(fp->max_velocity);
    pid_yaw.SetCmdMin(fp->max_velocity * (-1));
    break;
  default:
    ROS_WARN("Unknown PID parameters received.");
    break;
  }

}


void initialize_link(string id, ros::NodeHandle n_)
{
  string topicname;
  topicname = id;
  topicname.append("/base_pose_ground_truth");
  sub_stage = n_.subscribe(topicname.c_str(), 0, GazeboCallback);

  topicname = id;
  topicname.append("/position_control_pids");
  pid_parameters = n_.subscribe(topicname.c_str(), 0, PositionControlPIDCallback );

  topicname = id;
  topicname.append(string("/cmd_vel"));
  stagevel_pub = n_.advertise<geometry_msgs::Twist> (topicname.c_str(), 0);

  topicname =uavID;
  topicname.append("/ual_state");
  state_pub = n_.advertise<catec_msgs::UALStateStamped> (topicname.c_str(), 0);

  pid_x_windup = 0;
  pid_y_windup = 0;
  pid_z_windup = 0;
  pid_x_sat = 2.0;
  pid_y_sat = 2.0;
  pid_z_sat = 2.0;
  pid_x.Init(5,pid_x_windup,2,0.0,0.0,pid_x_sat,pid_x_sat*(-1));
  pid_y.Init(5,pid_y_windup,2,0.0,0.0,pid_y_sat,pid_y_sat*(-1));
  pid_z.Init(1.0,pid_z_windup,0.0,0.0,0.0,pid_z_sat,pid_z_sat*(-1));
  pid_yaw.Init(2.5,0.0,0.0,0.0,0.0,2.0,-2.0);



  //Temporal hasta que funcione todo.

}







