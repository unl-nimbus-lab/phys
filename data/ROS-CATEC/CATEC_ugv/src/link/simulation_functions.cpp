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

using namespace geometry_msgs;

extern double amax,vmax,anglemax;
bool init = true;

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

/*!
 * \brief Generate a Velocity Vector for send to UAV Simulator.
 */
Twist Update(Twist *res)
{
   res->linear.x = 0;
   res->linear.y = 0;
   res->linear.z = 0;
   res->angular.x = 0;
   res->angular.y = 0;
   res->angular.z = 0;

   Pose last_pose_cmd_;
   last_pose_cmd_.position.x = lastControlReferenceRw.c_reference_rw.position.x;
   last_pose_cmd_.position.y = lastControlReferenceRw.c_reference_rw.position.y;

   //   last_pose_cmd_.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, lastControlReferenceRw.c_reference_rw.heading);

   geometry_msgs::Pose p = last_pose_cmd_;
   geometry_msgs::Pose actual = lastPose;
   geometry_msgs::Vector3 vec_desp,vec_desp_unit;
   double distance;

   vec_desp.x = p.position.x-actual.position.x;
   vec_desp.y = p.position.y-actual.position.y;

   distance = sqrt(pow(vec_desp.x,2) + pow(vec_desp.y,2));

   vec_desp_unit.x = vec_desp.x/distance;
   vec_desp_unit.y = vec_desp.y/distance;

   geometry_msgs::Twist t;

   if(lastControlReferenceRw.c_reference_rw.cruise>0 && lastControlReferenceRw.c_reference_rw.cruise<(distance*3))
   {
      t.linear.x = lastControlReferenceRw.c_reference_rw.cruise*vec_desp_unit.x;
      t.linear.y = lastControlReferenceRw.c_reference_rw.cruise*vec_desp_unit.y;
   }
   else {
      t.linear.x = vec_desp.x*2;
      t.linear.y = vec_desp.y*2;
   }

   t.angular.z = atan2(lastControlReferenceRw.c_reference_rw.position.y - actual.position.y,
                       lastControlReferenceRw.c_reference_rw.position.x - actual.position.x);

   t.angular.z = t.angular.z + 1.57;
   t.angular.x = t.angular.y  = 0;

   geometry_msgs::Vector3 vout = limitva(t.linear, lastVel.linear, 1/timerate, vmax, amax);
   t.linear.x = vout.x;
   t.linear.y = vout.y;

   res->linear = t.linear;
   res->angular =t.angular;
   return *res;
}


