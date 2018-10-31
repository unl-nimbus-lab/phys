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
#include <iostream>

//#pragma pack(4)
#include <Interfaces.h>
#include <CUDPServer.h>
#include <std_msgs/Float64.h>
#include <catec_msgs/UALStateStamped.h>

#define PI 3.1415926535897932384626433832795028841971693993751058f
#define PORT 5224

#define MAX_OBJECT 15

#pragma(1)

using namespace std;
using namespace catec_msgs;

ros::Publisher testbed_state_pub[MAX_OBJECT];
ros::Publisher testbed_altimeter_pub[MAX_OBJECT];
IUavState received_state;
UALStateStamped send_state;
std_msgs::Float64 altimeterData;

CUDPServer udp_server;
int main(int argc, char** argv)
{
   struct sockaddr_in si_other;
   unsigned int slen = sizeof(si_other);

   ros::init(argc,argv,"qnx_hb_state_receiver");

   ros::NodeHandle n;
   string topicname;
   string sensorTopicname;
   char buf[5];

   for(int i=1;i<=MAX_OBJECT;i++)
   {
      topicname = "qnx_hb_state_";
      sensorTopicname = "qnx_hb_altimeter_";
      sprintf(buf,"%d",i);
      topicname.append(buf);
      sensorTopicname.append(buf);
      testbed_state_pub[i-1] = n.advertise<UALStateStamped> (
               topicname.c_str(), 0);
      testbed_altimeter_pub[i-1] = n.advertise<std_msgs::Float64> (
               sensorTopicname.c_str(), 0);
   }

   udp_server.connect(PORT);

   if(udp_server.s==-1)
   {
      cerr << "Error starting udp server." << endl;
      return -1;
   }

   ros::AsyncSpinner spinner(1);
   spinner.start();
   while(ros::ok())
   {

      if(!udp_server.receive(&received_state,sizeof(IUavState),
                             (sockaddr*)&si_other,(socklen_t*)&slen))
      {
         cerr << "Testbed_State: Error reading udp datagram..." << endl;
      }
        //TODO: Se cambia el número máximo de elementos a encontrar.
      else if(received_state.uavId.uiUavId <= MAX_OBJECT &&
              received_state.uavId.uiUavId > 0)
      {
         //cerr << "Received data size: "<< sizeof(IUavState) << endl;
         //cerr << "Received state from Interface: "<< received_state.posRel.dXRel
         //<<" UavID: "<< received_state.posRel.dYRel  << endl;
         send_state.header.stamp = ros::Time::now();
         send_state.header.frame_id = "qnx_hb_state_receiver";

         send_state.ual_state.dynamic_state.position.x = received_state.posRel.dXRel;
         send_state.ual_state.dynamic_state.position.y = received_state.posRel.dYRel;
         send_state.ual_state.dynamic_state.position.z = received_state.posRel.dZRel;
         //TODO: Warning the order was worse. This change could affect to other modules
         send_state.ual_state.dynamic_state.orientation.x = received_state.attitude.dPhiEuler;
         send_state.ual_state.dynamic_state.orientation.y = received_state.attitude.dThetaEuler;
         send_state.ual_state.dynamic_state.orientation.z = received_state.attitude.dPsiEuler;

         send_state.ual_state.dynamic_state.velocity.x = received_state.velEarth.dVelNorth;
         send_state.ual_state.dynamic_state.velocity.y = received_state.velEarth.dVelWest;
         send_state.ual_state.dynamic_state.velocity.z = received_state.velEarth.dVelUp;

         send_state.ual_state.flying_state = received_state.flyingState.uiFlyingState;

         testbed_state_pub[received_state.uavId.uiUavId-1].publish(send_state);

         altimeterData.data = received_state.hAgl.dHAgl;
         testbed_altimeter_pub[received_state.uavId.uiUavId-1].publish(altimeterData);
      }
      else
      {
         cerr << "Testbed_State: state received from unknow uav... Interface: "
              << received_state.state.uiIdInterface <<" UavID: "
              << received_state.uavId.uiUavId  << endl;
      }

   }
   udp_server.disconnect();
   return 0;
}
