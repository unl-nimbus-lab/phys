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
#include <catec_msgs/UALStateStamped.h>
#include <visualization_msgs/Marker.h>
// Conversion includes
#include<geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
// TF publication includes
#include <tf/transform_broadcaster.h>
#include "rotateOP/Quaternion.h"

//pixels
#define PI 3.1415926535897932384626433832795028841971693993751058f
using namespace catec_msgs;
using namespace rotateOp;
using namespace std;

//Transformations
tf::Transform ugvTF;
double counter = 0;

/*
 * Last UAL State (for actual Position)
 * */
UALStateStamped lastUALState;
void UALStateCallback(const catec_msgs::UALStateStamped::ConstPtr& s);

/*!
 * \brief Visualization publisher
 */
ros::Publisher vis_pub;
ros::Publisher vis_world;

/*!
 * \brief The number of the UAV node. Its range will be 0 to 10.
 */
string uavID;
int uavIntID;

int main(int argc, char** argv)
{
    if (argc < 1)
    {
        cout << "This program has two input parameters.\n"<<
                "The first input parameter is the number of the UAV." << endl;
        return -1;
    }

    // The UAV ID is stored in a global variable
    uavID="ugv_";
    uavID.append(string(argv[1]));

    try
    {
        uavIntID = boost::lexical_cast<int>(argv[1]);
    }
    catch(boost::bad_lexical_cast const&)
    {
        perror("The first argument is not a number.");
        return 1;
    }

    string topicname;
    topicname = uavID;
    topicname.append("/ual_state");

    ros::init(argc,argv,"dynamic_tf");

    ros::NodeHandle n;

    ros::Subscriber subState = n.subscribe(topicname, 1, UALStateCallback);
    vis_pub = n.advertise<visualization_msgs::Marker>(
                "visualization_uav", 0 );
    vis_world= n.advertise<visualization_msgs::Marker>(
                "visualization_world", 0 );

    ros::AsyncSpinner spinner_(2);
    spinner_.start();
    while(ros::ok())
    {
        sleep(1);
    }
}

void UALStateCallback(const catec_msgs::UALStateStamped::ConstPtr& s)
{
    lastUALState = *s;
    Quaternion q_s2;
    q_s2.fromEuler(lastUALState.ual_state.dynamic_state.orientation.x ,
                   lastUALState.ual_state.dynamic_state.orientation.y,
                   lastUALState.ual_state.dynamic_state.orientation.z,
                   TransformationTypes::EULER123);

    static tf::TransformBroadcaster br;
    ugvTF.setOrigin( tf::Vector3(lastUALState.ual_state.dynamic_state.position.x,
                                 lastUALState.ual_state.dynamic_state.position.y,
                                 lastUALState.ual_state.dynamic_state.position.z) );
    tf::Quaternion orientation;
    orientation.setX(q_s2.getX());
    orientation.setY(q_s2.getY());
    orientation.setZ(q_s2.getZ());
    orientation.setW(q_s2.getW());
    ugvTF.setRotation(orientation);

    br.sendTransform(tf::StampedTransform(ugvTF, ros::Time::now(),
                                          "map", "base_ugv"));

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/base_ugv";
    marker.header.stamp = ros::Time();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://CATEC_ugv/Media/models/cars/Lincoln/lincoln.dae";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 1.57;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker.mesh_use_embedded_materials = true;
    vis_pub.publish( marker );

    visualization_msgs::Marker markerScenario;
    markerScenario.header.frame_id = "/map";
    markerScenario.header.stamp = ros::Time();
    markerScenario.id = 0;
    markerScenario.type = visualization_msgs::Marker::MESH_RESOURCE;
    markerScenario.mesh_resource = "package://CATEC_uav/Media/models/scenario5/testbedTrakingLona.dae";
    markerScenario.action = visualization_msgs::Marker::ADD;
    markerScenario.pose.position.x = 0;
    markerScenario.pose.position.y = 0;
    markerScenario.pose.position.z = 0;
    markerScenario.pose.orientation.x = 0.0;
    markerScenario.pose.orientation.y = 0.0;
    markerScenario.pose.orientation.z = 0.0;
    markerScenario.pose.orientation.w = 1.0;
    markerScenario.scale.x = 1;
    markerScenario.scale.y = 1;
    markerScenario.scale.z = 1;
    markerScenario.mesh_use_embedded_materials = true;
    vis_world.publish( markerScenario );
}
