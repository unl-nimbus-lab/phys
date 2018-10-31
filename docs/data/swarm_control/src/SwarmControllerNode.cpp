/**
 *  This source file implements the SwarmControllerNode class, which is also a
 *  Node class via enhancement.
 *
 *  Version: 1.0.0
 *  Created on: 30/10/2016
 *  Modified on: 30/10/2016
 *  Author: Rafael Gomes Braga (faerugb@gmail.com)
 *  Maintainer: Rafael Gomes Braga (faerugb@gmail.com)
 */

#include "SwarmControllerNode.h"


SwarmControllerNode::SwarmControllerNode(ros::NodeHandle *nh)
  : Node(nh, 10)
{
    enableControl_ = false;
    dx_ = 0.0;
    dy_ = 0.0;
    initialDeltasCalculated_ = false;
    migrationPoint_.x = 0.0;
    migrationPoint_.y = 0.0;
    r1_ = 1.0;
    r2_ = 1.0;
    r3_ = 1.0;
    r4_ = 1.0;
    ros::param::get("swarm_controller_node/uav_id", id_);

    mavros_state_sub_ = nh->subscribe("mavros/state", 1, &SwarmControllerNode::mavrosStateCb, this);
    migration_point_sub_ = nh->subscribe("/migration_point", 1, &SwarmControllerNode::migrationPointCb, this);
    global_position_sub_ = nh->subscribe("mavros/global_position/global", 1, &SwarmControllerNode::globalPositionCb, this);
    odom_sub_ = nh->subscribe("mavros/local_position/odom", 1, &SwarmControllerNode::odomCb, this);
    enable_control_sub_ = nh->subscribe("/enable_control", 1, &SwarmControllerNode::enableControlCb, this);
    uavs_odom_sub_ = nh->subscribe("/uavs_odom", 1, &SwarmControllerNode::uavsOdomCb, this);
    uav_odom_pub_ = nh->advertise<swarm_control::OdometryWithUavId>("/uavs_odom", 10);
    cmd_vel_pub_ = nh->advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);


    v1_pub_ = nh->advertise<geometry_msgs::Point>("v1", 10);
    v2_pub_ = nh->advertise<geometry_msgs::Point>("v2", 10);
    v3_pub_ = nh->advertise<geometry_msgs::Point>("v3", 10);
    v4_pub_ = nh->advertise<geometry_msgs::Point>("v4", 10);
    vRes_pub_ = nh->advertise<geometry_msgs::Point>("vRes", 10);
}

SwarmControllerNode::~SwarmControllerNode()
{
    mavros_state_sub_.shutdown();
    migration_point_sub_.shutdown();
    global_position_sub_.shutdown();
    odom_sub_.shutdown();
    enable_control_sub_.shutdown();
    uavs_odom_sub_.shutdown();
    uav_odom_pub_.shutdown();
    cmd_vel_pub_.shutdown();

    v1_pub_.shutdown();
    v2_pub_.shutdown();
    v3_pub_.shutdown();
    v4_pub_.shutdown();
    vRes_pub_.shutdown();
}

void SwarmControllerNode::controlLoop()
{
    // Print information on screen
    int chosenUav = 2;
    if ( id_ == chosenUav )
    {
        char tab2[1024];
        strncpy(tab2, mode_.c_str(), sizeof(tab2));
        tab2[sizeof(tab2) - 1] = 0;
        //ROS_INFO("UAV %d: \n Mode = %s | enableControl = %d \n Position = (%f, %f) | Objective = (%f , %f)",
          //       id_, tab2, enableControl_, position_.x, position_.y, migrationPoint_.x, migrationPoint_.y);
    }

    // Calculate each rule's influence
    geometry_msgs::Point v1 = rule1();
    geometry_msgs::Point v2 = rule2();
    geometry_msgs::Point v3 = rule3();
    geometry_msgs::Point v4 = rule4();
    geometry_msgs::Point vRes;


    if ( id_ == chosenUav )
    {
        //ROS_INFO("v1 = (%f, %f, %f)", v1.x, v1.y, v1.z);
        //ROS_INFO("v2 = (%f, %f, %f)", v2.x, v2.y, v2.z);
        //ROS_INFO("v3 = (%f, %f, %f)", v3.x, v3.y, v3.z);
        //ROS_INFO("v4 = (%f, %f, %f)", v4.x, v4.y, v4.z);
    }


    // Combine the rules
    vRes.x = v1.x + v2.x + v3.x + v4.x;
    vRes.y = v1.y + v2.y + v3.y + v4.y;
    vRes.z = v1.z + v2.z + v3.z + v4.z;


    // Limit vRes
    double norm = sqrt( pow( (vRes.x), 2 ) +
                     pow( (vRes.y), 2 ) +
                     pow( (vRes.z), 2 ) );
    if ( norm >= MAXVEL )
    {
        vRes.x *= (MAXVEL / norm);
        vRes.y *= (MAXVEL / norm);
        vRes.z *= (MAXVEL / norm);
    }


    if ( id_ == 0 )
    {
        //ROS_INFO("vRes = (%f, %f, %f)", vRes.x, vRes.y, vRes.z);
    }


    // Publish velocity for diagnostics purpose
    v1_pub_.publish(v1);
    v2_pub_.publish(v2);
    v3_pub_.publish(v3);
    v4_pub_.publish(v4);
    vRes_pub_.publish(vRes);




    // PUBLISH VELOCITY
    if ( enableControl_ == true )
    {
        publishVelocity(vRes.x, vRes.y);
    }

    publishUavOdom();


    // TESTE: Imprimir os neighbors para ver se o robô está detectando corretamente
    /*int n = neighbors_.size();
    if ( n > 0 )
    {
        ROS_INFO("UAV %d neighbors:", id_);
        for ( int i(0); i < n; i++ )
        {
            if ( neighbors_[i].id != id_ )
            {
                ROS_INFO("[\n\tid: %d,\n\tx: %f,\n\ty: %f,\n\tz: %f,\n\tvx: %f,\n\tvy: %f,\n\tvz: %f\n]",
                         neighbors_[i].id,
                         neighbors_[i].odom.pose.pose.position.x,
                         neighbors_[i].odom.pose.pose.position.y,
                         neighbors_[i].odom.pose.pose.position.z,
                         neighbors_[i].odom.twist.twist.linear.x,
                         neighbors_[i].odom.twist.twist.linear.y,
                         neighbors_[i].odom.twist.twist.linear.z );
            }
        }
    }*/
}

void SwarmControllerNode::publishUavOdom()
{
    swarm_control::OdometryWithUavId msg;

    msg.id = id_;
    msg.odom.pose.pose.position.x = odom_.pose.pose.position.x;
    msg.odom.pose.pose.position.y = odom_.pose.pose.position.y;
    msg.odom.pose.pose.position.z = odom_.pose.pose.position.z;
    msg.odom.pose.pose.orientation.x = odom_.pose.pose.orientation.x;
    msg.odom.pose.pose.orientation.y = odom_.pose.pose.orientation.y;
    msg.odom.pose.pose.orientation.z = odom_.pose.pose.orientation.z;
    msg.odom.pose.pose.orientation.w = odom_.pose.pose.orientation.w;
    msg.odom.twist.twist.linear.x = odom_.twist.twist.linear.x;
    msg.odom.twist.twist.linear.y = odom_.twist.twist.linear.y;
    msg.odom.twist.twist.linear.z = odom_.twist.twist.linear.z;

    uav_odom_pub_.publish(msg);
}

void SwarmControllerNode::publishVelocity( double velX, double velY )
{
    geometry_msgs::TwistStamped msg;

    msg.twist.linear.x = velX;
    msg.twist.linear.y = velY;

    cmd_vel_pub_.publish(msg);
}

void SwarmControllerNode::mavrosStateCb(const mavros_msgs::StateConstPtr &msg)
{
    if(msg->mode == std::string("CMODE(0)"))
        return;
    //ROS_INFO("I heard: [%s] [%d] [%d]", msg->mode.c_str(), msg->armed, msg->guided);
    mode_ = msg->mode;
    guided_ = msg->guided==128;
    armed_ = msg->armed==128;
}

void SwarmControllerNode::migrationPointCb( const geometry_msgs::PointConstPtr &msg )
{
    migrationPoint_.x = msg->x;
    migrationPoint_.y = msg->y;
}

void SwarmControllerNode::globalPositionCb( const sensor_msgs::NavSatFix &msg )
{
    // Quando a primeira mensagem de GPS chegar, calcular dx_ e dy_
    if ( !initialDeltasCalculated_ )
    {
        double originLat, originLon;
        ros::param::get("swarm_controller_node/origin_lat", originLat);
        ros::param::get("swarm_controller_node/origin_lon", originLon);
        double lat = msg.latitude;
        double lon = msg.longitude;
        dx_ = haversines( originLat, originLon, originLat, lon );
        dy_ = haversines( originLat, originLon, lat, originLon );
        if ( (lon - originLon) < 0 ) dx_ *= -1;
        if ( (lat - originLat) < 0 ) dy_ *= -1;
        initialDeltasCalculated_ = true;
    }
}

void SwarmControllerNode::odomCb( const nav_msgs::OdometryConstPtr &msg )
{
    // Get UAV odometry from the received message
    odom_.pose.pose.position.x = msg->pose.pose.position.x + dx_;
    odom_.pose.pose.position.y = msg->pose.pose.position.y + dy_;
    odom_.pose.pose.position.z = msg->pose.pose.position.z;
    odom_.pose.pose.orientation.x = msg->pose.pose.orientation.x;
    odom_.pose.pose.orientation.y = msg->pose.pose.orientation.y;
    odom_.pose.pose.orientation.z = msg->pose.pose.orientation.z;
    odom_.pose.pose.orientation.w = msg->pose.pose.orientation.w;
    odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
    odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
    odom_.twist.twist.linear.z = msg->twist.twist.linear.z;

    // Publish to tf
    std::stringstream ss;
    ss << "/uav" << id_ << "/base_link";
    pose_br_.sendTransform(
        tf::StampedTransform(
            tf::Transform(
                tf::Quaternion(
                    odom_.pose.pose.orientation.x,
                    odom_.pose.pose.orientation.y,
                    odom_.pose.pose.orientation.z,
                    odom_.pose.pose.orientation.w
                ),
                tf::Vector3(
                    odom_.pose.pose.position.x,
                    odom_.pose.pose.position.y,
                    odom_.pose.pose.position.z
                )
            ), ros::Time::now(), "world", ss.str()
        )
    );
}

void SwarmControllerNode::enableControlCb( const std_msgs::BoolConstPtr &msg )
{
    enableControl_ = msg->data;
}

void SwarmControllerNode::uavsOdomCb( const swarm_control::OdometryWithUavIdConstPtr &msg )
{   
    ROS_INFO( "UAV %d reporting. Received new msg:\n[\n\tid: %d,\n\tx: %f,\n\ty: %f,\n\tz: %f,\n\tvx: %f,\n\tvy: %f,\n\tvz: %f\n]",
             id_,
             msg->id,
             msg->odom.pose.pose.position.x,
             msg->odom.pose.pose.position.y,
             msg->odom.pose.pose.position.z,
             msg->odom.twist.twist.linear.x,
             msg->odom.twist.twist.linear.y,
             msg->odom.twist.twist.linear.z );
    // Only consider messages that came from neighbors
    if ( msg->id != id_ )
    {
        ROS_INFO( "UAV %d reporting. Entering if. Number of neighbors is %d.", id_, neighbors_.size() );
        bool isNew = true;

        // Loop through the neighbors array
        for ( int i = 0; i < neighbors_.size(); i++ )
        {
            ROS_INFO( "UAV %d reporting. Entering neighbor %d. Id is %d.", id_, i, neighbors_[i].id );
            // Update existing neighbors...
            if ( msg->id == neighbors_[i].id )
            {
                ROS_INFO( "UAV %d reporting. Neighbor already exists. Updating.", id_ );
                neighbors_[i].odom.pose.pose.position.x = msg->odom.pose.pose.position.x;
                neighbors_[i].odom.pose.pose.position.y = msg->odom.pose.pose.position.y;
                neighbors_[i].odom.pose.pose.position.z = msg->odom.pose.pose.position.z;
                neighbors_[i].odom.pose.pose.orientation.x = msg->odom.pose.pose.orientation.x;
                neighbors_[i].odom.pose.pose.orientation.y = msg->odom.pose.pose.orientation.y;
                neighbors_[i].odom.pose.pose.orientation.z = msg->odom.pose.pose.orientation.z;
                neighbors_[i].odom.pose.pose.orientation.w = msg->odom.pose.pose.orientation.w;
                neighbors_[i].odom.twist.twist.linear.x = msg->odom.twist.twist.linear.x;
                neighbors_[i].odom.twist.twist.linear.y = msg->odom.twist.twist.linear.y;
                neighbors_[i].odom.twist.twist.linear.z = msg->odom.twist.twist.linear.z;

                isNew = false;
                break;
            }
        }

        // ...or add new neighbors
        if ( isNew == true )
        {
            ROS_INFO( "UAV %d reporting. Neighbor is new. Adding.", id_ );
            swarm_control::OdometryWithUavId odomWithUavId;
            odomWithUavId.id = msg->id;
            odomWithUavId.odom.pose.pose.position.x = msg->odom.pose.pose.position.x;
            odomWithUavId.odom.pose.pose.position.y = msg->odom.pose.pose.position.y;
            odomWithUavId.odom.pose.pose.position.z = msg->odom.pose.pose.position.z;
            odomWithUavId.odom.pose.pose.orientation.x = msg->odom.pose.pose.orientation.x;
            odomWithUavId.odom.pose.pose.orientation.y = msg->odom.pose.pose.orientation.y;
            odomWithUavId.odom.pose.pose.orientation.z = msg->odom.pose.pose.orientation.z;
            odomWithUavId.odom.pose.pose.orientation.w = msg->odom.pose.pose.orientation.w;
            odomWithUavId.odom.twist.twist.linear.x = msg->odom.twist.twist.linear.x;
            odomWithUavId.odom.twist.twist.linear.y = msg->odom.twist.twist.linear.y;
            odomWithUavId.odom.twist.twist.linear.z = msg->odom.twist.twist.linear.z;
            neighbors_.push_back(odomWithUavId);
        }
    }

    ROS_INFO( "UAV %d reporting. New array:", id_ );
    int n = neighbors_.size();
    if ( n > 0 )
    {
        ROS_INFO("UAV %d neighbors:", id_);
        for ( int i(0); i < n; i++ )
        {
            if ( neighbors_[i].id != id_ )
            {
                ROS_INFO("[\n\tid: %d,\n\tx: %f,\n\ty: %f,\n\tz: %f,\n\tvx: %f,\n\tvy: %f,\n\tvz: %f\n]",
                         neighbors_[i].id,
                         neighbors_[i].odom.pose.pose.position.x,
                         neighbors_[i].odom.pose.pose.position.y,
                         neighbors_[i].odom.pose.pose.position.z,
                         neighbors_[i].odom.twist.twist.linear.x,
                         neighbors_[i].odom.twist.twist.linear.y,
                         neighbors_[i].odom.twist.twist.linear.z );
            }
        }
    }
}


// REYNOLDS RULES

// Rule 1: Flocking
geometry_msgs::Point SwarmControllerNode::rule1()
{
    geometry_msgs::Point v1;

    v1.x = 0.0;
    v1.y = 0.0;
    v1.z = 0.0;

    int n = neighbors_.size();

    if ( n > 0 )
    {
        geometry_msgs::Point centerOfMass;

        centerOfMass.x = 0.0;
        centerOfMass.y = 0.0;
        centerOfMass.z = 0.0;

        for ( int i(0); i < n; i++ )
        {
            if ( neighbors_[i].id != id_ )
            {
                centerOfMass.x += neighbors_[i].odom.pose.pose.position.x;
                centerOfMass.y += neighbors_[i].odom.pose.pose.position.y;
                centerOfMass.z += neighbors_[i].odom.pose.pose.position.z;
            }
        }

        centerOfMass.x *= ( 1 / n );
        centerOfMass.y *= ( 1 / n );
        centerOfMass.z *= ( 1 / n );

        v1.x = centerOfMass.x - odom_.pose.pose.position.x;
        v1.y = centerOfMass.y - odom_.pose.pose.position.y;
        v1.z = centerOfMass.z - odom_.pose.pose.position.z;
    }

    v1.x *= r1_;
    v1.y *= r1_;
    v1.z *= r1_;

    return v1;
}


// Rule 2: Collision Avoidance
geometry_msgs::Point SwarmControllerNode::rule2()
{
    geometry_msgs::Point v2;

    v2.x = 0.0;
    v2.y = 0.0;
    v2.z = 0.0;

    int n = neighbors_.size();

    if ( n > 0 )
    {
        for ( int i(0); i < n; i++ )
        {
            if ( neighbors_[i].id != id_ )
            {
                double d = sqrt( pow( (neighbors_[i].odom.pose.pose.position.x - odom_.pose.pose.position.x), 2 ) +
                                 pow( (neighbors_[i].odom.pose.pose.position.y - odom_.pose.pose.position.y), 2 ) +
                                 pow( (neighbors_[i].odom.pose.pose.position.z - odom_.pose.pose.position.z), 2 ) );

                if ( d < VISION_DISTANCE )
                {
                    double dif = VISION_DISTANCE - d;

                    geometry_msgs::Point v;

                    v.x = neighbors_[i].odom.pose.pose.position.x - odom_.pose.pose.position.x;
                    v.y = neighbors_[i].odom.pose.pose.position.y - odom_.pose.pose.position.y;
                    v.z = neighbors_[i].odom.pose.pose.position.z - odom_.pose.pose.position.z;

                    double vm = sqrt( pow( (v.x), 2 ) + pow( (v.y), 2 ) + pow( (v.z), 2 ) );

                    if ( vm < 0.01 ) vm = 0.1;

                    v.x /= vm;
                    v.y /= vm;
                    v.z /= vm;

                    v.x *= dif;
                    v.y *= dif;
                    v.z *= dif;

                    v2.x -= v.x;
                    v2.y -= v.y;
                    v2.z -= v.z;
                }
            }
        }
    }

    v2.x *= r2_;
    v2.y *= r2_;
    v2.z *= r2_;

    return v2;
}

// Rule 3: Velocity Matching
geometry_msgs::Point SwarmControllerNode::rule3()
{
    geometry_msgs::Point v3;

    v3.x = 0.0;
    v3.y = 0.0;
    v3.z = 0.0;

    int n = neighbors_.size();

    if ( n > 0 )
    {
        for ( int i(0); i < n; i++ )
        {
            if ( neighbors_[i].id != id_ )
            {
                v3.x += neighbors_[i].odom.twist.twist.linear.x;
                v3.y += neighbors_[i].odom.twist.twist.linear.y;
                v3.z += neighbors_[i].odom.twist.twist.linear.z;
            }
        }

        v3.x *= ( 1 / n );
        v3.y *= ( 1 / n );
        v3.z *= ( 1 / n );

        v3.x = v3.x - odom_.twist.twist.linear.x;
        v3.y = v3.y - odom_.twist.twist.linear.y;
        v3.z = v3.z - odom_.twist.twist.linear.z;
    }

    v3.x *= r3_;
    v3.y *= r3_;
    v3.z *= r3_;

    return v3;
}


// Rule 4: Migration
geometry_msgs::Point SwarmControllerNode::rule4()
{
    geometry_msgs::Point v4;

    v4.x = 0.0;
    v4.y = 0.0;
    v4.z = 0.0;

    v4.x = migrationPoint_.x - odom_.pose.pose.position.x;
    v4.y = migrationPoint_.y - odom_.pose.pose.position.y;
    v4.z = migrationPoint_.z - odom_.pose.pose.position.z;

    v4.x *= r4_;
    v4.y *= r4_;
    v4.z *= r4_;

    return v4;
}


// Returns the distance in meters between 2 points given their GPS coordinates
double SwarmControllerNode::haversines( double lat1, double lon1, double lat2, double lon2 )
{
    double a, c, d, dLat, dLon;
    int r = 6371000; // raio médio da Terra em metros
    // converter os ângulos para radianos:
    double degToRad = PI / 180.0;
    lat1 *= degToRad;
    lat2 *= degToRad;
    lon1 *= degToRad;
    lon2 *= degToRad;
    dLat = lat2 - lat1;
    dLon = lon2 - lon1;

    // fórmula de haversines:
    a = sin( dLat / 2 ) * sin( dLat / 2 ) +
               cos( lat1 ) * cos( lat2 ) *
               sin( dLon / 2 ) * sin( dLon / 2 );
    c = 2 * atan2( sqrt( a ), sqrt( 1 - a ) );
    d = r * c;

    return d;
}
