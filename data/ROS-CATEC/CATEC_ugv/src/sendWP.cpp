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
#include <catec_msgs/ControlReferenceRwStamped.h>
#include <catec_msgs/UALStateStamped.h>
#include <visualization_msgs/Marker.h>

using namespace std;

/*
 * Last UAL State (for actual Position)
 * */
catec_msgs::UALStateStamped lastUALState;
void UALStateCallback(const catec_msgs::UALStateStamped::ConstPtr& s);

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        cout << "This program has 3 input parameter.\n"<<
                "The first input parameter is the number of the UGV." <<
                "The second is the file path of the waypoints." << endl;
        return -1;
    }

    // The UAV ID is stored in a global variable
    string ugvID="ugv_";
    ugvID.append(string(argv[1]));
    int ugvIntID;
    try
    {
        ugvIntID = boost::lexical_cast<int>(argv[1]);
    }
    catch(boost::bad_lexical_cast const&)
    {
        perror("The first argument is not a number.");
        return 1;
    }

    visualization_msgs::Marker markerL;
    markerL.header.frame_id = "/map";
    markerL.header.stamp = ros::Time();
    markerL.id = 0;
    markerL.type = visualization_msgs::Marker::SPHERE_LIST;
    markerL.action = visualization_msgs::Marker::ADD;
    FILE *fp = NULL;
    float f1, f2, f3;
    vector<double> x, y, vel;
    fp = fopen(argv[2], "r");
    if(fp == NULL)
    {
        perror("The file does not exist.");
        return 1;
    }
    while(fscanf(fp, "%f %f %f\n", &f1, &f2, &f3) == 3){
        x.push_back(f1);
        y.push_back(f2);
        vel.push_back(f3);
        geometry_msgs::Point p;
        p.x = f1;
        p.y = f2;
        p.z = 0.1;
        markerL.points.push_back(p);
    }

    ros::init(argc,argv,"TrajectoryGuided");
    ros::NodeHandle n;

    string visT = "visualization_traj_";
    visT.append(string(argv[1]));
    ros::Publisher vis_pubT = n.advertise<visualization_msgs::Marker>(visT, 0 );
    markerL.pose.orientation.x = 0.0;
    markerL.pose.orientation.y = 0.0;
    markerL.pose.orientation.z = 0.0;
    markerL.pose.orientation.w = 1.0;
    markerL.scale.x = 0.05;
    markerL.scale.y = 0.05;
    markerL.scale.z = 0.05;
    markerL.color.r = 0.2f*ugvIntID;
    markerL.color.g = 0.14f*ugvIntID;
    markerL.color.b = 0.15f*(ugvIntID-5);
    markerL.color.a = 1.0;


    string topicname;
    topicname = ugvID;
    topicname.append("/control_references_rw");
    ros::Publisher wpPub =
            n.advertise<catec_msgs::ControlReferenceRwStamped>(topicname.c_str(), 0);
//    topicname = "uav_5";
//    topicname.append("/control_references_rw");
//    ros::Publisher wpPub_uav =
//            n.advertise<catec_msgs::ControlReferenceRwStamped>(topicname.c_str(), 0);
    visT = "visualization_current_wp_";
    visT.append(string(argv[1]));
    ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>(visT, 0 );

    topicname = ugvID;
    topicname.append("/ual_state");
    ros::Subscriber subState = n.subscribe(topicname, 1, UALStateCallback);

    int wpIndex = 0;
    ros::Rate r(25);
    while(ros::ok())
    {
        double dist = sqrt(pow((lastUALState.ual_state.dynamic_state.position.x - x[wpIndex]),2)+
                           pow((lastUALState.ual_state.dynamic_state.position.y - y[wpIndex]),2));
        //cout << "Distante to current wp : " << dist << endl;
        if(dist < 0.2)
        {
            if(wpIndex == x.size()-1)
            {
                wpIndex = 0;
            }
            else {
                wpIndex++;
            }
        }
        catec_msgs::ControlReferenceRwStamped wp;
        wp.c_reference_rw.cruise = vel[wpIndex];
        wp.c_reference_rw.position.x = x[wpIndex];
        wp.c_reference_rw.position.y = y[wpIndex];
        wpPub.publish(wp);
//        wp.c_reference_rw.cruise = 1.5;
//        wp.c_reference_rw.position.z = 0.7;
//        wp.c_reference_rw.heading = wp.c_reference_rw.position.x*10;
//        wpPub_uav.publish(wp);

//        cout << "WP sent : ("
//             << wp.c_reference_rw.position.x << ","
//             << wp.c_reference_rw.position.y<<") to "
//             << wp.c_reference_rw.cruise << endl;

        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time();
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = wp.c_reference_rw.position.x;
        marker.pose.position.y = wp.c_reference_rw.position.y;
        marker.pose.position.z = 0.1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = 0.2f*ugvIntID;
        marker.color.g = 0.14f*ugvIntID;
        marker.color.b = 0.15f*(ugvIntID-5);
        marker.color.a = 0.5;
        vis_pubT.publish( markerL );
        vis_pub.publish( marker );

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

void UALStateCallback(const catec_msgs::UALStateStamped::ConstPtr& s)
{
    lastUALState = *s;
}


