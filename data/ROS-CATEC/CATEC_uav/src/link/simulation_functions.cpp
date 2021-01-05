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
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <LinearMath/btQuaternion.h>

#define PI 3.1415926535897932384626433832795028841971693993751058f
#define LAND_Z 0.20
#define TAKE_OFF_Z 0.3

using namespace geometry_msgs;

extern double amax,vmax,anglemax;

geometry_msgs::Vector3 limitva(geometry_msgs::Vector3 vobjg, geometry_msgs::Vector3 vnowg, double timestep, double vmax, double amax){
  tf::Vector3 vout, aobj, a, vobj(vobjg.x, vobjg.y, vobjg.z), vnow(vnowg.x, vnowg.y, vnowg.z );

  aobj = (vobj -vnow)/timestep;
  if(aobj.length() > amax  ){
    a = amax * aobj.normalized();
  }
  else if(aobj.length() <= amax){
    a =  aobj;
  }

  vout = vnow + a*timestep;
  if(vout.length() > vmax)
  {
    vout = vout.normalized()*vmax;
  }

  geometry_msgs::Vector3 out;
  tf::vector3TFToMsg(vout, out);

  //Otherwise, it won't work.
  if( isnan(aobj.length()) || lastPose.position.z > 50 ){
    out = vobjg;
  }
  return out;
}


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
      ROS_INFO("CmdFlag receive, taking off...");
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


/*!
 * \brief Generate a Velocity Vector for send to UAV Simulator.
 */
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
  res->linear.z = 0;

  res->angular.x = 0;
  res->angular.y = 0;
  res->angular.z = 0;
  if(actual_state!=LANDED)
  {
    Pose last_pose_cmd_;
    last_pose_cmd_.position.x = lastControlReferenceRw.c_reference_rw.position.x;
    last_pose_cmd_.position.y = lastControlReferenceRw.c_reference_rw.position.y;
    last_pose_cmd_.position.z = lastControlReferenceRw.c_reference_rw.position.z;

    last_pose_cmd_.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, lastControlReferenceRw.c_reference_rw.heading);

    //ROS_INFO("Goal Position: x: %f y: %f z: %f", lastControlReferenceRw.c_reference_rw.position.x,lastControlReferenceRw.c_reference_rw.position.y,lastControlReferenceRw.c_reference_rw.position.z);

      geometry_msgs::Pose p = last_pose_cmd_;
      geometry_msgs::Pose actual = lastPose;
      geometry_msgs::Vector3 vec_desp,vec_desp_unit;
      double distance;

      vec_desp.x = p.position.x-actual.position.x;
      vec_desp.y = p.position.y-actual.position.y;
      vec_desp.z = p.position.z-actual.position.z;

      distance = sqrt(pow(vec_desp.x,2) + pow(vec_desp.y,2) + pow(vec_desp.z,2));

      vec_desp_unit.x = vec_desp.x/distance;
      vec_desp_unit.y = vec_desp.y/distance;
      vec_desp_unit.z = vec_desp.z/distance;

      /*double rad = sqrt((actual.position.x-p.position.x)*(actual.position.x-p.position.x) +
          (actual.position.y-p.position.y)*(actual.position.y-p.position.y));
      double angle = atan2(p.position.y-actual.position.y,p.position.x-actual.position.x);*/

      //ROS_INFO("PACtual: x: %f y: %f z: %f", lastPose.position.x,lastPose.position.y,lastPose.position.z);
      //ROS_INFO("Futura: x: %f y: %f z: %f", p.position.x,p.position.y,p.position.z);

      //ROS_INFO("Angle: %f UAV_ANGLE: %f", angle,tf::getYaw(actual.orientation));

      geometry_msgs::Twist t;

      if(lastControlReferenceRw.c_reference_rw.cruise>0 && lastControlReferenceRw.c_reference_rw.cruise<(distance*3))
      {
        t.linear.x = lastControlReferenceRw.c_reference_rw.cruise*vec_desp_unit.x;
        t.linear.y = lastControlReferenceRw.c_reference_rw.cruise*vec_desp_unit.y;
        t.linear.z = lastControlReferenceRw.c_reference_rw.cruise*vec_desp_unit.z;
      }
      else {
        t.linear.x = vec_desp.x*2;
        t.linear.y = vec_desp.y*2;
        t.linear.z = vec_desp.z*2;
      }

    /*	if(abs(p.position.z - actual.position.z) > 0.05)
      {
        if(p.position.z < actual.position.z)
        {
          t.linear.z = -0.5;
        }
        else
        {
          t.linear.z = 0.5;
        }
      }
      else
      {
        t.linear.z = 0;
      }*/

      if(actual_state!=LANDING && lastPose.position.z < TAKE_OFF_Z && t.linear.z < 0)
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
      }

      //ROS_INFO("Cmd Vel: x: %.3f  y: %.3f  z: %.3f", t.linear.x,t.linear.y,t.linear.z);
      //ROS_INFO("Actual : x: %.3f  y: %.3f  z: %.3f", lastVel.linear.x,lastVel.linear.y,lastVel.linear.z);
      //ROS_INFO("Cmd Vel: vmax: %f  amax: %f", vmax,amax);
      geometry_msgs::Vector3 vout = limitva(t.linear, lastVel.linear, 1/timerate, vmax, amax);

      //ROS_INFO("Limitad: x: %.3f  y: %.3f  z: %.3f", vout.x,vout.y,vout.z);
      //t.linear = vout;
      t.linear.z = vout.z;
      //vout.z = 0;
      t.linear.x = vout.x;
      t.linear.y = vout.y;

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
    res->linear = t.linear;
    res->angular.z =t.angular.z;

    }
    return *res;
}


