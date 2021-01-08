#include "channel_controller/channel_controller.h"
#include "channel_controller/bresenham.h"
#include <pluginlib/class_list_macros.h>
#include <string>
#include <nav_msgs/Path.h>
#include <angles/angles.h>
#include <visualization_msgs/MarkerArray.h>
#include <kobuki_msgs/Sound.h>
#include <kobuki_msgs/Led.h>
#include <boost/foreach.hpp>
#include <std_msgs/Empty.h>
#include <algorithm>
#define forEach BOOST_FOREACH

namespace channel_controller
{

PLUGINLIB_EXPORT_CLASS(channel_controller::ChannelController, nav_core::BaseLocalPlanner);

ChannelController::ChannelController() : tf_(NULL), costmap_ros_(NULL), current_waypoint_(0), use_laser_(true), use_costmap_(true)
{
}

ChannelController::~ChannelController()
{
}

void ChannelController::initialize(std::string name,
        tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    state_ = CSFollowChannel;
    safe_waypoint_state_ = SWSNone;

    waiting_for_obstacles_start_time_ = ros::Time(0);

    voronoi_.initializeEmpty(costmap_ros_->getCostmap()->getSizeInCellsX(),
            costmap_ros_->getCostmap()->getSizeInCellsY(), true);

    // Name is probably something like channel_controller::ChannelController
    // And our param then should be ChannelController
    size_t colon = name.find_last_of(":");
    std::string class_name = name.substr(colon + 1);
    if(colon == std::string::npos)
        class_name = name;

    ROS_INFO("Initializing ChannelController from ~/%s", class_name.c_str());
    ros::NodeHandle nhPriv("~/" + class_name);    // ~ = /move_base, our config should be in /move_base/name
    ros::NodeHandle nh;

    nhPriv.param("use_laser", use_laser_, true);
    nhPriv.param("use_costmap", use_costmap_, true);

    nhPriv.param("safe_waypoint_channel_width", safe_waypoint_channel_width_, 0.5);
    nhPriv.param("safe_waypoint_channel_width_at_max_tv", safe_waypoint_channel_width_at_max_tv_, 0.5);
    nhPriv.param("safe_channel_width", safe_channel_width_, 0.4);
    nhPriv.param("channel_score_da", channel_score_da_, 1.0);
    nhPriv.param("channel_score_dist", channel_score_dist_, 0.3);

    nhPriv.param("max_channel_length", max_channel_length_, 3.0);

    nhPriv.param("min_get_to_safe_dist_time", min_get_to_safe_dist_time_, 3.0);
    nhPriv.param("max_get_to_safe_dist_time", max_get_to_safe_dist_time_, 10.0);

    nhPriv.param("waypoint_reached_dist", waypoint_reached_dist_, 0.3);
    nhPriv.param("waypoint_reached_dist_at_max_tv", waypoint_reached_dist_at_max_tv_, 0.3);
    nhPriv.param("waypoint_reached_angle", waypoint_reached_angle_, 7.0);
    nhPriv.param("goal_reached_dist", goal_reached_dist_, 0.1);
    nhPriv.param("goal_reached_angle", goal_reached_angle_, 0.22);

    nhPriv.param("min_tv", min_tv_, 0.1);
    nhPriv.param("max_tv", max_tv_, 0.6);
    nhPriv.param("min_rv", min_rv_, 0.0);
    nhPriv.param("max_rv", max_rv_, 0.9);
    nhPriv.param("min_inplace_rv", min_inplace_rv_, 0.1);

    nhPriv.param("stopped_tv", stopped_tv_, 0.05);
    nhPriv.param("stopped_rv", stopped_rv_, 0.1);

    nhPriv.param("max_accel_tv", max_accel_tv_, 2.0);
    nhPriv.param("max_accel_rv", max_accel_rv_, 1.5);

    nhPriv.param("wait_for_obstacles_time", wait_for_obstacles_time_, -1.0);
    nhPriv.param("no_progress_hard_clear_time", no_progress_hard_clear_time_, -1.0);

    nhPriv.param("vis_max_dist", vis_max_dist_, 1.0);
    nhPriv.param("visualize_voronoi", visualize_voronoi_, false);

    ROS_INFO("use_laser: %d", use_laser_);
    ROS_INFO("use_costmap: %d", use_costmap_);
    ROS_INFO("safe_waypoint_channel_width: %f", safe_waypoint_channel_width_);
    ROS_INFO("safe_waypoint_channel_width_at_max_tv: %f", safe_waypoint_channel_width_at_max_tv_);
    ROS_INFO("safe_channel_width: %f", safe_channel_width_);
    ROS_INFO("channel_score_da: %f", channel_score_da_);
    ROS_INFO("channel_score_dist: %f", channel_score_dist_);
    ROS_INFO("min_get_to_safe_dist_time: %f", min_get_to_safe_dist_time_);
    ROS_INFO("max_get_to_safe_dist_time: %f", max_get_to_safe_dist_time_);
    ROS_INFO("waypoint_reached_dist: %f", waypoint_reached_dist_);
    ROS_INFO("waypoint_reached_dist_at_max_tv: %f", waypoint_reached_dist_at_max_tv_);
    ROS_INFO("waypoint_reached_angle: %f", waypoint_reached_angle_);
    ROS_INFO("goal_reached_dist: %f", goal_reached_dist_);
    ROS_INFO("goal_reached_angle: %f", goal_reached_angle_);
    ROS_INFO("min_tv: %f", min_tv_);
    ROS_INFO("max_tv: %f", max_tv_);
    ROS_INFO("min_rv: %f", min_rv_);
    ROS_INFO("max_rv: %f", max_rv_);
    ROS_INFO("min_inplace_rv: %f", min_inplace_rv_);
    ROS_INFO("stopped_tv: %f", stopped_tv_);
    ROS_INFO("stopped_rv: %f", stopped_rv_);
    ROS_INFO("max_accel_tv: %f", max_accel_tv_);
    ROS_INFO("max_accel_rv: %f", max_accel_rv_);
    ROS_INFO("wait_for_obstacles_time: %f", wait_for_obstacles_time_);
    ROS_INFO("no_progress_hard_clear_time: %f", no_progress_hard_clear_time_);
    ROS_INFO("vis_max_dist: %f", vis_max_dist_);
    ROS_INFO("visualize_voronoi: %d", visualize_voronoi_);

    pub_markers_ = nhPriv.advertise<visualization_msgs::MarkerArray>("channel_markers", 1);
    pub_status_marker_ = nhPriv.advertise<visualization_msgs::Marker>("drive_channel_status", 1);
    pub_local_plan_ = nhPriv.advertise<nav_msgs::Path>("local_plan", 1);

    pub_sound_ = nh.advertise<kobuki_msgs::Sound>("mobile_base/commands/sound", 1);
    pub_led_ = nh.advertise<kobuki_msgs::Led>("mobile_base/commands/led1", 3);

    pub_call_clear_ = nh.advertise<std_msgs::Empty>("/call_clear", 1);

    sub_odom_ = nh.subscribe("odom", 2, &ChannelController::odometryCallback, this);
    sub_laser_ = nh.subscribe("base_scan", 3, &ChannelController::laserCallback, this);

    //updateVoronoi();

    srand48(time(NULL));
}

bool ChannelController::updateVoronoi()
{
    bool voronoi_ok = true;
    costmap_ = costmap_ros_->getCostmap();
    ROS_ASSERT(costmap_->getSizeInCellsX() == voronoi_.getSizeX());
    ROS_ASSERT(costmap_->getSizeInCellsY() == voronoi_.getSizeY());

    std::vector<IntPoint> obstacles;
    if(use_costmap_) {
        for(unsigned int x = 0; x < costmap_->getSizeInCellsX(); x++) {
            for(unsigned int y = 0; y < costmap_->getSizeInCellsY(); y++) {
                if(costmap_->getCost(x, y) >= costmap_2d::LETHAL_OBSTACLE) { // lethal and unknown
                    obstacles.push_back(IntPoint(x, y));
                }
            }
        }
    }
    float validRangesFrac = 0.0;
    if(use_laser_) {
        double dt = (ros::Time::now() - last_laser_.header.stamp).toSec();
        //printf("LASER AGE: %f\n", dt);
        if(dt > 0.2) {
            ROS_ERROR("%s: Laser too old - age is: %f", __func__, dt);
            return false;
        }
        // get tf from laser to costmap/voronoi frame
        tf::StampedTransform transform;
        try {
            if(!tf_->waitForTransform(costmap_ros_->getGlobalFrameID(),
                    last_laser_.header.frame_id, last_laser_.header.stamp, ros::Duration(0.1))) {
                ROS_ERROR("Current Laser TF not available");
                return false;
            }
            tf_->lookupTransform(costmap_ros_->getGlobalFrameID(),
                    last_laser_.header.frame_id, last_laser_.header.stamp, transform);
        } catch(tf::TransformException & e) {
            ROS_ERROR("%s: TF Error: %s", __func__, e.what());
            return false;
        }
        unsigned int validRanges = 0;   // to check if we got actual data, i.e. Go drive!
        unsigned int endIdx = last_laser_.ranges.size();
        endIdx = std::min(endIdx, 1000u); // start/end from index filter params
        for(unsigned int i = 80; i < endIdx; i++) {
            if(last_laser_.ranges[i] <= last_laser_.range_min || last_laser_.ranges[i] >= last_laser_.range_max)
                continue;
            validRanges++;
            double da = last_laser_.angle_min + last_laser_.angle_increment * i;
            tf::Vector3 pt(last_laser_.ranges[i] * cos(da), last_laser_.ranges[i] * sin(da), 0.0);
            tf::Vector3 ptCostmap = transform * pt;
            unsigned int ix, iy;
            if(costmap_->worldToMap(ptCostmap.x(), ptCostmap.y(), ix, iy)) {
                obstacles.push_back(IntPoint(ix, iy));
            }
        }
        if(last_laser_.ranges.size() > 0)
            validRangesFrac = (double)validRanges/(double)last_laser_.ranges.size();
    }
    // If there are no obstacles, wait as a precaution (assume there always will be _some_)
    // Will stop in actual free space!
    if(wait_for_obstacles_time_ > 0.0 && obstacles.empty() && validRangesFrac < 0.2) {
        if(waiting_for_obstacles_start_time_ == ros::Time(0)) {
            // first time we got in here
            waiting_for_obstacles_start_time_ = ros::Time::now();

            ROS_WARN("No obstacles in local costmap. Starting to wait for %fs",
                    wait_for_obstacles_time_);

            kobuki_msgs::Sound warn_sound;
            warn_sound.value = kobuki_msgs::Sound::ERROR;
            //usleep(500*1000);
            pub_sound_.publish(warn_sound);
            voronoi_ok = false;
        } else if(ros::Time::now() - waiting_for_obstacles_start_time_ <=
                ros::Duration(wait_for_obstacles_time_)) {
            ROS_WARN_THROTTLE(1.0, "No obstacles in local costmap. Waiting %.1fs/%.1fs before going.",
                    (ros::Time::now() - waiting_for_obstacles_start_time_).toSec(),
                    wait_for_obstacles_time_);
            voronoi_ok = false;
        } // else: over the time, but still no obstacles
        // keep the start time active until we receive obstacles,
        // but continue going anyways as the wait time is over
    }
    // obstacles found, reset waiting_for_obstacles_start_time_
    if(!obstacles.empty())
        waiting_for_obstacles_start_time_ = ros::Time(0);

    voronoi_.exchangeObstacles(obstacles);
    voronoi_.update(true);
    //voronoi_.prune(); // FIXME This only does voronoi stuff, not distance related.
    if(visualize_voronoi_)
        visualizeVoronoi();

    return voronoi_ok;
}

void ChannelController::visualizeVoronoi()
{
    ROS_ASSERT(costmap_->getSizeInCellsX() == voronoi_.getSizeX());
    ROS_ASSERT(costmap_->getSizeInCellsY() == voronoi_.getSizeY());

    // nothing to send to no one
    if(pub_markers_.getNumSubscribers() == 0)
        return;

    visualization_msgs::MarkerArray channelMarkers;
    visualization_msgs::Marker voronoiMarker;
    voronoiMarker.header.frame_id = costmap_ros_->getGlobalFrameID();
    voronoiMarker.header.stamp = ros::Time(0);
    voronoiMarker.ns = "voronoi";
    voronoiMarker.id = 0;
    voronoiMarker.type = visualization_msgs::Marker::SPHERE_LIST;
    voronoiMarker.action = visualization_msgs::Marker::ADD;
    voronoiMarker.pose.orientation.w = 1.0;
    voronoiMarker.scale.x = costmap_->getResolution() * sqrt(2.0);
    voronoiMarker.scale.y = costmap_->getResolution() * sqrt(2.0);
    voronoiMarker.scale.z = costmap_->getResolution() * sqrt(2.0);
    voronoiMarker.frame_locked = false;
    geometry_msgs::Point cellPoint;
    cellPoint.z = - costmap_->getResolution() * sqrt(2.0)/2.0;
    std_msgs::ColorRGBA cellColor;
    cellColor.a = 1.0;
    for(unsigned int x = 0; x < costmap_->getSizeInCellsX(); x++) {
        for(unsigned int y = 0; y < costmap_->getSizeInCellsY(); y++) {
            float dist = voronoi_.getDistance(x, y);
            dist *= costmap_->getResolution();   // now in meters
            costmap_->mapToWorld(x, y, cellPoint.x, cellPoint.y);
            voronoiMarker.points.push_back(cellPoint);

            if(dist == -INFINITY) {
                cellColor.r = 1.0;
                cellColor.g = 0.0;
                cellColor.b = 1.0;
            } else if(dist == INFINITY) {
                cellColor.r = 0.0;
                cellColor.g = 1.0;
                cellColor.b = 0.0;
            } else {
                if(dist > vis_max_dist_) {
                    cellColor.r = cellColor.g = cellColor.b = 1.0;
                } else {
                    // make those slightly darker then max dist to distinguish
                    // vis max dist regions
                    cellColor.r = cellColor.g = cellColor.b = 0.9 * dist / vis_max_dist_;
                }
            }

            voronoiMarker.colors.push_back(cellColor);
        }
    }
    channelMarkers.markers.push_back(voronoiMarker);
    pub_markers_.publish(channelMarkers);
}

bool ChannelController::isGoalReached()
{
    if(ros::Time::now() - last_odom_.header.stamp > ros::Duration(0.5)) {
        ROS_ERROR_THROTTLE(1.0, "isGoalReached:: Last odom is too old: %f - not at goal.",
                (ros::Time::now() - last_odom_.header.stamp).toSec());
        return false;
    }
    double cur_tv = last_odom_.twist.twist.linear.x;
    double cur_rv = last_odom_.twist.twist.angular.z;
    if(current_waypoint_ >= global_plan_.size() &&
            fabs(cur_tv) < stopped_tv_ && fabs(cur_rv) < 2.0*min_inplace_rv_) {
        ROS_INFO("ChannelController: Goal Reached!");
        kobuki_msgs::Sound sound;
        sound.value = kobuki_msgs::Sound::CLEANINGEND;
        pub_sound_.publish(sound);
        return true;
    } else {
        //ROS_INFO("Not at goal. At wp %d/%zu vel %f %f", current_waypoint_, global_plan_.size(),
        //        cur_tv, cur_rv);
    }
    return false;
}

bool ChannelController::setPlan(const std::vector<geometry_msgs::PoseStamped> & plan)
{
    // check if the new plan has the same goal
    bool sameGoal = false;
    if(!global_plan_.empty() && !plan.empty()) {
        geometry_msgs::PoseStamped currentGoal = global_plan_.back();
        geometry_msgs::PoseStamped newGoal = plan.back();
        if(currentGoal.header.frame_id == newGoal.header.frame_id) {    // ignore time for fixed frames
            tf::Pose currentGoalPose;
            tf::Pose newGoalPose;
            tf::poseMsgToTF(currentGoal.pose, currentGoalPose);
            tf::poseMsgToTF(newGoal.pose, newGoalPose);
            tf::Pose relPose = currentGoalPose.inverseTimes(newGoalPose);
            if(relPose.getOrigin().length() < 0.05 &&
                    fabs(tf::getYaw(relPose.getRotation())) < angles::from_degrees(5.0)) {
                sameGoal = true;
            }
        }
    }

    global_plan_ = plan;
    current_waypoint_ = 0;
    if(false && sameGoal && state_ == CSGetToSafeWaypointDist) {
        ROS_WARN("New plan for same goal - keeping state CSGetToSafeWaypointDist");
        state_ = CSGetToSafeWaypointDist;
    } else {
        state_ = CSFollowChannel;
    }

    if(!sameGoal) {
        last_progress_time_ = ros::Time::now();
    }

    // localize now to verify this makes sense somehow and return upon that.
    return localizeGlobalPlan(current_waypoint_);
}

bool ChannelController::currentWaypointReached() const
{
    if(local_plan_.empty())
        return true;

    tf::Stamped<tf::Pose> robot_pose;
    if(!costmap_ros_->getRobotPose(robot_pose))
        return false;

    tf::Stamped<tf::Pose> currentWaypoint = local_plan_.front();
    tf::Pose toWaypoint = robot_pose.inverseTimes(currentWaypoint);

    double cur_tv = last_odom_.twist.twist.linear.x;
    double distThreshold = waypoint_reached_dist_ + (waypoint_reached_dist_at_max_tv_ - waypoint_reached_dist_) *
        straight_up(cur_tv, min_tv_, max_tv_);
    double angleThreshold = waypoint_reached_angle_;
    if(local_plan_.size() == 1) {   // only 1 wp left -> goal wp
        distThreshold = goal_reached_dist_;
        if((ros::Time::now() - goal_turn_start_time_) < ros::Duration(5.0)) {
            angleThreshold = goal_reached_angle_/2.0;
        } else {
            angleThreshold = goal_reached_angle_;
        }
    }

    if(state_ != CSGoalTurn) {
        // Setting state to CSGoalTurn determined that this
        // already was true once, from then on we don't care
        // (latched goal approach)
        // This must not be checked in that case as CSGoalTurn
        // only turns to goal, but never approaches again.
        if(toWaypoint.getOrigin().length() > distThreshold)
            return false;
    }
    if(fabs(tf::getYaw(toWaypoint.getRotation())) > angleThreshold)
        return false;

    return true;
}

bool ChannelController::localizeGlobalPlan(unsigned int start_index)
{
    // FIXME test this with different frames for map/odom
    nav_msgs::Path localPlanMsg;
    localPlanMsg.header.stamp = ros::Time(0);
    localPlanMsg.header.frame_id = costmap_ros_->getGlobalFrameID();

    local_plan_.clear();
    if(global_plan_.empty()) {
        pub_local_plan_.publish(localPlanMsg);
        return false;
    }

    tf::StampedTransform transform;
    // purposely ignoreing the global_plan_.header.stamp here as we assume that the global_plan_
    // frame is a fixed frame and the controller's something like odom.
    // We thus want the most current transformation into our frame for the global_plan_.
    try {
        tf_->lookupTransform(costmap_ros_->getGlobalFrameID(),
                global_plan_.front().header.frame_id, ros::Time(0), transform);
    } catch(tf::TransformException & e) {
        ROS_ERROR("%s: TF Error: %s", __func__, e.what());
        pub_local_plan_.publish(localPlanMsg);
        return false;
    }

    // we'll assume all frame_id are the same in global plan
    for(unsigned int i = start_index; i < global_plan_.size(); i++) {
        tf::Pose global_pose;
        tf::poseMsgToTF(global_plan_.at(i).pose, global_pose);

        tf::Stamped<tf::Pose> local_pose;
        local_pose.frame_id_ = costmap_ros_->getGlobalFrameID();
        local_pose.stamp_ = transform.stamp_;
        local_pose.setData(transform * global_pose);
        local_plan_.push_back(local_pose);

        if(pub_local_plan_.getNumSubscribers() > 0) {
            geometry_msgs::PoseStamped local_pose_msg;
            tf::poseStampedTFToMsg(local_pose, local_pose_msg);
            localPlanMsg.poses.push_back(local_pose_msg);
        }
    }

    pub_local_plan_.publish(localPlanMsg);
    return true;
}


DriveChannel ChannelController::computeChannel(tf::Pose from_pose, tf::Pose to_pose, double clearance_dist) const
{
    ROS_ASSERT(costmap_->getSizeInCellsX() == voronoi_.getSizeX());
    ROS_ASSERT(costmap_->getSizeInCellsY() == voronoi_.getSizeY());

    DriveChannel channel;
    channel.from_pose_ = from_pose;
    channel.min_dist_ = HUGE_VAL;

    int start_x, start_y;
    int target_x, target_y;
    costmap_->worldToMapNoBounds(from_pose.getOrigin().x(), from_pose.getOrigin().y(),
            start_x, start_y);
    costmap_->worldToMapNoBounds(to_pose.getOrigin().x(), to_pose.getOrigin().y(),
            target_x, target_y);
    int trace_x = start_x;
    int trace_y = start_y;
    for(Bresenham b(start_x, start_y, target_x, target_y); b.hasNext(); b.advance()) {
        if(b.cur_x() < 0 || b.cur_y() < 0)
            break;
        if(b.cur_x() >= (int)voronoi_.getSizeX() || b.cur_y() >= (int)voronoi_.getSizeY())
            break;
        float dist = voronoi_.getDistance(b.cur_x(), b.cur_y());
        dist *= costmap_->getResolution();
        if(dist == -INFINITY)   // +INFINITY Ok
            break;
        if(dist < clearance_dist)
            break;
        if(dist < channel.min_dist_)
            channel.min_dist_ = dist;
        trace_x = b.cur_x();
        trace_y = b.cur_y();
    }
    double dx = costmap_->getResolution() * (trace_x - start_x);
    double dy = costmap_->getResolution() * (trace_y - start_y);

    tf::Pose deltaPose = from_pose.inverseTimes(to_pose);
    tf::Pose deltaChannelLength(deltaPose.getRotation(),
            deltaPose.getOrigin().normalized() * hypot(dx, dy));
    channel.to_pose_ = from_pose * deltaChannelLength;

    return channel;
}

visualization_msgs::Marker ChannelController::createChannelMarkers(
        const std::vector<DriveChannel> & channels, double min_good_dist,
        int best_idx) const
{
    visualization_msgs::Marker channelMarker;
    channelMarker.header.stamp = ros::Time(0);
    channelMarker.header.frame_id = costmap_ros_->getGlobalFrameID();
    channelMarker.ns = "channels";
    channelMarker.id = 2;
    channelMarker.type = visualization_msgs::Marker::LINE_LIST;
    channelMarker.action = visualization_msgs::Marker::ADD;
    channelMarker.pose.orientation.w = 1.0;
    channelMarker.scale.x = 0.01;
    channelMarker.lifetime = ros::Duration(0);
    channelMarker.frame_locked = false;

    int index = 0;
    forEach(const DriveChannel & channel, channels) {
        // rotation of 90 for half of width and the end to show width
        double dir = channel.direction();
        double min_dist = channel.min_dist_;
        if(min_dist > 2.0 * std::max(costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY())) {
            min_dist = 2.0 * std::max(costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
        }
        tf::Vector3 half_width(0.0, min_dist, 0.0);
        tf::Vector3 half_width_other(0.0, -min_dist, 0.0);
        tf::Pose side_offset(tf::createQuaternionFromYaw(0.0), half_width);
        tf::Pose side_offset_other(tf::createQuaternionFromYaw(0.0), half_width_other);
        tf::Pose channel_dir(tf::createQuaternionFromYaw(dir));
        tf::Pose z = tf::Pose(channel.from_pose_.getRotation()) * channel_dir * side_offset;
        tf::Pose z2 = tf::Pose(channel.from_pose_.getRotation()) * channel_dir * side_offset_other;

        geometry_msgs::Point pt;
        pt.x = channel.from_pose_.getOrigin().x();
        pt.y = channel.from_pose_.getOrigin().y();
        pt.z = 0.0;
        channelMarker.points.push_back(pt);
        //ROS_INFO_STREAM("X CHANNEL TO " << pt << " LENGTH " << channel.length());
        pt.x = channel.to_pose_.getOrigin().x();
        pt.y = channel.to_pose_.getOrigin().y();
        channelMarker.points.push_back(pt);
        if(channel.length() > 0) {
            pt.x = channel.to_pose_.getOrigin().x() + z2.getOrigin().x();
            pt.y = channel.to_pose_.getOrigin().y() + z2.getOrigin().y();
            channelMarker.points.push_back(pt);
            pt.x = channel.to_pose_.getOrigin().x() + z.getOrigin().x();
            pt.y = channel.to_pose_.getOrigin().y() + z.getOrigin().y();
            channelMarker.points.push_back(pt);
        }
        std_msgs::ColorRGBA col;
        col.r = 0.7;
        col.g = 0.0;
        col.b = 0.0;
        col.a = 1.0;
        if(channel.length() >= min_good_dist) {
            col.r = 0.0;
            col.g = 0.7;
            col.b = 0.0;
        }
        if(index == best_idx) {
            col.r = 0.0;
            col.g = 0.0;
            col.b = 1.0;
        }
        channelMarker.colors.push_back(col);
        channelMarker.colors.push_back(col);
        if(channel.length() > 0) {
            channelMarker.colors.push_back(col);
            channelMarker.colors.push_back(col);
        }
        index++;
    }

    return channelMarker;
}

visualization_msgs::Marker ChannelController::createPoseMarker(const tf::Pose & pose,
        double r, double g, double b,
        const std::string & ns, int id) const
{
    visualization_msgs::Marker poseMarker;
    poseMarker.header.stamp = ros::Time(0);
    poseMarker.header.frame_id = costmap_ros_->getGlobalFrameID();
    poseMarker.ns = ns;
    poseMarker.id = id;
    poseMarker.type = visualization_msgs::Marker::SPHERE;
    poseMarker.action = visualization_msgs::Marker::ADD;
    tf::poseTFToMsg(pose, poseMarker.pose);
    poseMarker.scale.x = 0.1;
    poseMarker.scale.y = 0.1;
    poseMarker.scale.z = 0.1;
    poseMarker.color.r = r;
    poseMarker.color.g = g;
    poseMarker.color.b = b;
    poseMarker.color.a = 1.0;
    poseMarker.lifetime = ros::Duration(0);
    poseMarker.frame_locked = false;

    return poseMarker;
}

double ChannelController::straight_up(double x, double a, double b) const
{
    // linearly go up from 0 to 1 between a and b
    if(x <= a)
        return 0.0;
    if(x >= b)
        return 1.0;
    return (x - a)/(b - a);
}

double ChannelController::straight_down(double x, double a, double b) const
{
    // linearly go down from 1 to 0 between a and b
    if(x <= a)
        return 1.0;
    if(x >= b)
        return 0.0;
    return (b - x)/(b - a);
}

void ChannelController::odometryCallback(const nav_msgs::Odometry & odom)
{
    last_odom_ = odom;
}

void ChannelController::laserCallback(const sensor_msgs::LaserScan & laser)
{
    last_laser_ = laser;
}

void ChannelController::limitTwist(geometry_msgs::Twist & cmd_vel) const
{
    // Whats the point of this fn: Basically prevent the impossible
    // When we set a tv/rv traj, we still drive on that even if one
    // of the values doesn't work
    // just that we drive on it slower.
    // That's what the result should be. Must make sure that always happens, especially
    // when avoiding obstacles/braking
    // Very important for collision avoidance.

    // FIXME Zerg Hack, this should be done properly
    // assuming no max_accel_rv
    //
    // If tv is small, we are either fast and braking hard
    // or we are slow and accelerating
    // In both cases, there is no need to limit.
    if(fabs(cmd_vel.linear.x) < 2.0 * min_tv_) {
        return;
    }

    double cur_tv = last_odom_.twist.twist.linear.x;
    double cur_rv = last_odom_.twist.twist.angular.z;

    if(cur_tv * cmd_vel.linear.x < 0) {
        if(fabs(cur_tv) < min_tv_) {    // special case near zero, no error
            cur_tv = 0.0;
        } else {
            ROS_ERROR_THROTTLE(1.0, "%s: TV changes sign, cur: %f, target: %f - cannot handle this, not limiting!",
                    __func__, cur_tv, cmd_vel.linear.x);
        }
        return;
    }

    // TODO dead reckoning option
    //double dt = (ros::Time::now() - last_cmd_vel_time_).toSec();  // estimated dt for commands
    double dt = (ros::Time::now() - last_odom_.header.stamp).toSec();  // estimated dt for commands
    if(dt < 0) {
        ROS_ERROR_THROTTLE(1.0, "%s: Got odometry from the future, dt: %f, setting 0",
                __func__, dt);
    }
    // If we haven't send commands for a while, we'll get an initial "kick"
    // with a large dt, this should be OK
    double dx_max = max_accel_tv_ * dt;
    double dth_max = max_accel_rv_ * dt;

    // FIXME later: stopping for obstacle? -> test and increase accel limits or not limit
    // if the channel is too short.

    double twist_scale = 1.0;
    if(fabs(cmd_vel.linear.x - cur_tv) > dx_max) {
        twist_scale = dx_max/fabs(cmd_vel.linear.x - cur_tv);
    }
    //ROS_DEBUG("twist_scale tv: %f ", twist_scale);
    if(max_accel_rv_ > 0 && fabs(cmd_vel.angular.z - cur_rv) > dth_max) {
        twist_scale = std::min(twist_scale, dth_max/fabs(cmd_vel.angular.z - cur_rv));
    }
    //ROS_DEBUG("twist_scale after rv: %f ", twist_scale);

    // scale both for same trajectory
    if(twist_scale < 1.0) {
        // braking = we set a lower absolute speed OR we even reverse the speed
        if(fabs(cmd_vel.linear.x) < fabs(cur_tv) || (cmd_vel.linear.x * cur_tv) < 0) {
            ROS_DEBUG_THROTTLE(1.0, "Not scaling twist to accel limits as we are braking: old tv: %f target tv: %f", cur_tv, cmd_vel.linear.x);
        } else {
            geometry_msgs::Twist new_cmd_vel = cmd_vel;
            new_cmd_vel.linear.x = cur_tv + twist_scale * 
                (cmd_vel.linear.x - cur_tv);
            // We need to get as close to the target traj given by tv/rv as possible
            double ideal_scaled_rv = new_cmd_vel.linear.x/cmd_vel.linear.x * cmd_vel.angular.z;
            if(max_accel_rv_ <= 0) {
                new_cmd_vel.angular.z = ideal_scaled_rv;
            } else {
                // how close can we get
                if(fabs(ideal_scaled_rv - cur_rv) > dth_max) {  // can't reach, take largest step
                    if(ideal_scaled_rv < cur_rv) {
                        new_cmd_vel.angular.z = cur_rv - dth_max;
                    } else {
                        new_cmd_vel.angular.z = cur_rv + dth_max;
                    }
                } else {
                    new_cmd_vel.angular.z = ideal_scaled_rv;
                }
            }

            //new_cmd_vel.angular.z = cur_rv + twist_scale *
            //    (cmd_vel.angular.z - cur_rv);
            //    //twist_scale * cmd_vel.angular.z;

            // never scale at/near min vels
            // never scale to/near min vels

            //// check mins
            //// FIXME Not relevant when scaling here?
            //// -> we'd have a large delta anyways...
            //if(fabs(cmd_vel.linear.x) < stopped_tv_) {
            //    if(fabs(new_cmd_vel.angular.z) < min_inplace_rv_) {
            //        new_cmd_vel.angular.z = sign(cmd_vel.angular.z) * min_inplace_rv_;
            //    }
            //} else {
            //    if(fabs(new_cmd_vel.linear.x) < min_tv_)
            //        new_cmd_vel.linear.x = sign(cmd_vel.linear.x) * min_tv_;
            //    if(fabs(new_cmd_vel.angular.z) < min_rv_)
            //        new_cmd_vel.angular.z = sign(cmd_vel.angular.z) * min_rv_;
            //}
            /*ROS_DEBUG("%s: dt: %f, dx_max: %f, dth_max: %f", __func__,
                    dt, dx_max, dth_max);
            ROS_DEBUG("%s: cur tv: %f target tv: %f, cur rv: %f, target rv: %f", __func__,
                    cur_tv, cmd_vel.linear.x, cur_rv, cmd_vel.angular.z);*/
            ROS_DEBUG("%s: Scaling cmd vel from %f, %f -> %f %f", __func__,
                    cmd_vel.linear.x, cmd_vel.angular.z,
                    new_cmd_vel.linear.x, new_cmd_vel.angular.z);
            cmd_vel = new_cmd_vel;
        }
    }
}

bool ChannelController::computeVelocityForChannel(const DriveChannel & channel, geometry_msgs::Twist & cmd_vel, ScopedVelocityStatus & status) const
{
    // TODO make sure that channel length are until free and not until waypoint
    // Maybe only if there is no more waypoints near it limit for corner racing
    // BUT maybe we just stop quickly???
    // goal approach is scaled with speed already
    // TODO final speed advise is safty 
    // 2. think about channel selection as in what do we want/need
    // a) for close quarters
    // b) for high speed open area (later, also need to check how global plans look there)
    // c) dont tune too much towards global plan - dynamic obst are NOT in there and the dist counts
    // -- more: think about why channels with dist arent on global plan, can we pause and observe?
    // 3. redo speed selection with fast and safe

    ROS_ASSERT(!local_plan_.empty());
    cmd_vel = geometry_msgs::Twist();   // init to 0

    double channel_dir = channel.direction();
    double channel_length = channel.length();
    double channel_width = 2.0 * channel.min_dist_;

    // the channel should have originated at robot_pose
    tf::Pose relToWp = channel.from_pose_.inverseTimes(local_plan_.front());
    // FIXME later: maybe look ahead if next waypoints are on the same channels: GO!

    // channel way different, stop first, then turn in place
    double cur_tv = last_odom_.twist.twist.linear.x;
    double cur_rv = last_odom_.twist.twist.angular.z;
    if(fabs(channel_dir) > angles::from_degrees(45)) {
        //double cur_rv = last_odom_.twist.twist.angular.z;
        if(fabs(cur_tv) < stopped_tv_) {
            // Just turn full. We're stopped and way in the wrong direction
            cmd_vel.angular.z = max_rv_ * sign(channel_dir);
            status.setChannelTurnToChannel(channel_dir, cur_tv, cur_rv);
        } else {    // else: stop (0)
            status.setChannelStopToTurn(channel_dir, cur_tv, cur_rv);
        }

        return true;
    }

    // kinda steer towards channel
    cmd_vel.angular.z = sign(channel_dir) * (min_rv_ + (max_rv_ - min_rv_) *
            straight_up(fabs(channel_dir), angles::from_degrees(0.0), angles::from_degrees(60.0)));

    // FIXME later use width as main scaling? relate to robot?

    // TODO align those to max values with time

    // speed up if there is space
    double channel_length_tv_scale = straight_up(channel_length, 0.3, 2.0);

    // go slower forward if we aren't pointing in the right direction
    double bad_direction_tv_scale =
        straight_down(fabs(channel_dir), angles::from_degrees(10.0), angles::from_degrees(60.0));

    // go slower when narrow
    tf::Pose zero;
    bool in_bounds = true;
    double around_dist = getDistanceAtPose(zero, &in_bounds);
    double close_to_obstacles_scale = 1;
    if (in_bounds)
    {
    	close_to_obstacles_scale = 0.33 + 0.66 * straight_up(around_dist, 0.3, 0.6);
    }

	double channel_width_tv_scale = straight_up(channel_width,
                safe_waypoint_channel_width_, safe_waypoint_channel_width_at_max_tv_);

    // for now ignore close to wp unless goal.
    double close_to_goal_tv_scale = 1.0;
    double close_to_goal_rv_scale = 1.0;
    if(local_plan_.size() == 1) {   // going for goal
        double dist_to_wp = relToWp.getOrigin().length();
        double angle_to_wp = atan2(relToWp.getOrigin().y(), relToWp.getOrigin().x());
        close_to_goal_tv_scale = straight_up(dist_to_wp,
                2.0 * goal_reached_dist_, 5.0 * goal_reached_dist_);
        close_to_goal_rv_scale = straight_up(fabs(angle_to_wp),
                goal_reached_angle_, 5.0 * goal_reached_angle_);
    }

    std::string tv_scale_reason = "none";
    double tv_scale = bad_direction_tv_scale;
    if(tv_scale < 1.0)
        tv_scale_reason = "bad_direction";
    tv_scale = std::min(tv_scale, channel_length_tv_scale);
    if(tv_scale == channel_length_tv_scale)
        tv_scale_reason = "channel_length";
    tv_scale = std::min(tv_scale, channel_width_tv_scale);
    if(tv_scale == channel_width_tv_scale)
        tv_scale_reason = "channel_width";
    tv_scale = std::min(tv_scale, close_to_obstacles_scale);
    if(tv_scale == close_to_goal_tv_scale)
        tv_scale_reason = "close_to_obstacles";
    tv_scale = std::min(tv_scale, close_to_goal_tv_scale);
    if(tv_scale == close_to_goal_tv_scale)
        tv_scale_reason = "close_to_goal";
    if(tv_scale >= 0.99)
        tv_scale_reason = "none";

    std::string rv_scale_reason = "none";
    double rv_scale = close_to_goal_rv_scale;
    if(rv_scale < 1.0)
        rv_scale_reason = "close_to_goal";
    if(rv_scale >= 0.99)
        rv_scale_reason = "none";

    // TODO the relationship and scaling for rv/tv should more use
    // absolute values matched to the max values
    // not just a factor of the max, i.e. high max rv will turn to rapidly
    // when tv max isn't that high, i.e. the combination isn't good.
    cmd_vel.linear.x = min_tv_ + (max_tv_ - min_tv_) * tv_scale;
    cmd_vel.angular.z *= 0.2 + 0.8 * rv_scale;

    // Let's make sure we're not below the min values
    double tv_min_fact = fabs(cmd_vel.linear.x)/min_tv_;
    // If we're too slow ahead is only OK if we're at least turning
    // somewhat quickly
    if(tv_min_fact < 1.0 && fabs(cmd_vel.angular.z) < angles::from_degrees(10)) {
        // scale up so we get at least one higher.
        cmd_vel.linear.x /= tv_min_fact;
        cmd_vel.angular.z /= tv_min_fact;
    }
    if(fabs(cmd_vel.linear.x) < stopped_tv_ && fabs(cmd_vel.angular.z) < min_inplace_rv_) {
        ROS_DEBUG("%s: tv almost 0 and rv below min_inplace_rv_ -> increasing rv", __func__);
        cmd_vel.angular.z = sign(cmd_vel.angular.z) * min_inplace_rv_;
        cmd_vel.linear.x = sign(cmd_vel.linear.x) * stopped_tv_;    // drive at least stopped tv
    }

    //limitTwist(cmd_vel);

    status.setChannelFollowChannel(channel_dir, cur_tv, cur_rv, tv_scale_reason, rv_scale_reason);

    return true;
}

void ChannelController::computeVelocityForSafeChannel(
        const DriveChannel & channel,
        geometry_msgs::Twist & cmd_vel, ScopedVelocityStatus & status,
        bool turn_in) const
{
    cmd_vel = geometry_msgs::Twist();   // init to 0

    double channel_dir = channel.direction();

    bool backwards = false;
    // check if channel is behind us - go backwards
    if(fabs(channel_dir) > angles::from_degrees(90)) {
        backwards = true;
        // substract 90 deg to get equivalent forward channel
        channel_dir = sign(channel_dir) * (fabs(channel_dir) - angles::from_degrees(90.0));
        // switch dir as we're going backwards
        channel_dir *= -1.0;
    }
    if(!turn_in)
        channel_dir *= -1.0;

    cmd_vel.linear.x = min_tv_;
    if(backwards)
        cmd_vel.linear.x *= -1.0;

    // kinda steer towards channel slowly
    cmd_vel.angular.z = sign(channel_dir) * (min_rv_ + 0.1 * (max_rv_ - min_rv_) *
            straight_up(fabs(channel_dir), angles::from_degrees(10.0), angles::from_degrees(90.0)));
}

bool ChannelController::handleGoalTurn(geometry_msgs::Twist & cmd_vel,
        const tf::Stamped<tf::Pose> & robotPose, double distToTarget,
        ScopedVelocityStatus & status)
{
    if(state_ != CSGoalTurn) {
        if(local_plan_.size() != 1) // either already goal reached or not approach goal wp
            return false;

        if(distToTarget > goal_reached_dist_)   // approaching goal but not near enough yet
            return false;

        goal_turn_start_time_ = ros::Time::now();
    }

    state_ = CSGoalTurn;   // latch this once we're in here - stay

    // we are at the goal point
    // as currentWaypointReached didn't trigger: we're not turned to it

    tf::Stamped<tf::Pose> currentWaypoint = local_plan_.front();
    tf::Pose toWaypoint = robotPose.inverseTimes(currentWaypoint);

    double da = angles::normalize_angle(tf::getYaw(toWaypoint.getRotation())); 
    double curTv = last_odom_.twist.twist.linear.x;
    double curRv = last_odom_.twist.twist.angular.z;
    if(fabs(curTv) < stopped_tv_) {
        cmd_vel.angular.z = sign(da) * (min_inplace_rv_ + (max_rv_ - min_inplace_rv_) *
                straight_up(fabs(da), angles::from_degrees(10.0), angles::from_degrees(90.0)));
        status.setAtGoalPosTurnToGoal(da, curTv, curRv);
    } else {    // else: stop (0)
        status.setAtGoalPosStopToTurn(da, curTv, curRv);
    }

    return true;
}

double ChannelController::getDistanceAtPose(const tf::Pose & pose, bool* in_bounds) const
{
    // determine current dist
    int pose_x, pose_y;
    costmap_->worldToMapNoBounds(pose.getOrigin().x(), pose.getOrigin().y(),
            pose_x, pose_y);
    if(pose_x < 0 || pose_y < 0 ||
            pose_x >= (int)voronoi_.getSizeX() || pose_y >= (int)voronoi_.getSizeY()) {
        if(in_bounds == NULL) {
            // only warn if in_bounds isn't checked externally
            ROS_WARN_THROTTLE(1.0, "%s: Pose out of voronoi bounds (%.2f, %.2f) = (%d, %d)", __func__,
                    pose.getOrigin().x(), pose.getOrigin().y(), pose_x, pose_y);
        } else {
            *in_bounds = false;
        }
        return 0.0;
    } 
    if(in_bounds)  {
        *in_bounds = true;
    }
    float dist = voronoi_.getDistance(pose_x, pose_y);
    dist *= costmap_->getResolution();
    return dist;
}

int ChannelController::getToSafeWaypoint(geometry_msgs::Twist & cmd_vel,
        const tf::Pose & robotPose, const tf::Pose & relativeTarget,
        ScopedVelocityStatus & status)
{
    if(state_ != CSGetToSafeWaypointDist)   // we're not active
        return 0;

    ros::Duration activeTime = ros::Time::now() - get_to_safe_waypoint_start_time_;
    if(activeTime > ros::Duration(max_get_to_safe_dist_time_))
        return -1;

    // determine current dist
    bool pose_in_bounds;
    double dist = getDistanceAtPose(robotPose, &pose_in_bounds);
    if(!pose_in_bounds) {
        ROS_ERROR("%s: Robot pose not in bounds", __func__);    // shouldn't happen
        return -1;
    }

    // stay active if activeTime < min_get_to_safe_dist_time_ or
    // we're not at safe waypoint dist
    if(dist >= safe_waypoint_channel_width_/2.0 &&
            activeTime >= ros::Duration(min_get_to_safe_dist_time_)) {
        // out of obst and executed long enough -> we're done
        state_ = CSFollowChannel;
        return -2;
    }

    // actual behavior - choose channels.
    std::vector<DriveChannel> safe_channels = computeChannels(robotPose, relativeTarget,
            safe_channel_width_/2.0);
    if(safe_channels.empty()) {
        ROS_ERROR_THROTTLE(1.0, "Could find no safe_channels - recovery needed");
        return -1;
    }

    int best_idx = -1;
    if(safe_waypoint_state_ == SWSBestChannel)
        best_idx = evaluateSafeChannels(safe_channels, false);
    else if(safe_waypoint_state_ == SWSBestBackwardsChannelTurnIn ||
            safe_waypoint_state_ == SWSBestBackwardsChannelTurnOut)
        best_idx = evaluateSafeChannels(safe_channels, true);
    if(best_idx < 0) {
        ROS_ERROR_THROTTLE(1.0, "Could not find best safe channel");
        return -1;
    }

    visualization_msgs::MarkerArray channelMarkers;
    channelMarkers.markers.push_back(createChannelMarkers(safe_channels, 0.0, best_idx));
    pub_markers_.publish(channelMarkers);

    if(safe_waypoint_state_ == SWSBestChannel ||
            safe_waypoint_state_ == SWSBestBackwardsChannelTurnIn)
        computeVelocityForSafeChannel(safe_channels[best_idx], cmd_vel, status, true);
    else if(safe_waypoint_state_ == SWSBestBackwardsChannelTurnOut)
        computeVelocityForSafeChannel(safe_channels[best_idx], cmd_vel, status, false);

    status.setGetToSafeWaypoint(dist, activeTime.toSec(), safe_waypoint_state_);

    return 1;
}

std::vector<DriveChannel> ChannelController::computeChannels(const tf::Pose & robotPose,
        const tf::Pose & relativeTarget, double minDist) const
{
    std::vector<DriveChannel> channels;
    for(double da = -M_PI; da <= M_PI - angles::from_degrees(5.0); da += angles::from_degrees(5.0)) {
        tf::Pose rotDir(tf::createQuaternionFromYaw(da));
        // point in da, along up to the whole costmap length
        tf::Pose relativeTargetDir(relativeTarget.getRotation(),
                relativeTarget.getOrigin().normalized() *
                    max_channel_length_);
        tf::Pose rotTarget = robotPose * rotDir * relativeTargetDir;

        DriveChannel channel = computeChannel(robotPose, rotTarget, minDist);
        channel.da_ = da;
        if(channel.length() > 0)
            channels.push_back(channel);
    }
    return channels;
}

double ChannelController::computeChannelScore(double da, double dist) const
{
    // Scoring is based on da and min_dist:
    // small da + small dist
    // - Score by dist and a bit da (already there keep from obst + steer)
    // small da + large dist
    // - Score by dist and a bit da (already there keep from obst + steer)
    // large da + small dist
    // - Score by da (try to get there first)
    // large da + large dist
    // - Score by da (try to get there first)
    // => Make this a smooth transition
    double da_score = 0.5 * (cos(fabs(da)) + 1.0);  // [0..1] based on cos -> 1.0 for da = 0.0
    //// keeping dist_score influence very conservative for now.
    //// A 20% wider channel gets a bit more score.
    //double dist_score = straight_up(dist, safe_channel_width_/2.0, 1.2 * safe_waypoint_channel_width_/2.0);
    double dist_score = straight_up(dist, safe_waypoint_channel_width_/2.0, safe_waypoint_channel_width_at_max_tv_/2.0);
    double score = channel_score_da_ * da_score + channel_score_dist_ * dist_score;
    return score;
}

int ChannelController::evaluateChannels(const std::vector<DriveChannel> & channels,
        double distToTarget) const
{
    int best_idx = -1;
    double best_score = -1.0;
    ROS_DEBUG("dist to target is: %f", distToTarget);
    for(unsigned int channel_idx = 0; channel_idx < channels.size(); channel_idx++) {
        const DriveChannel & channel = channels.at(channel_idx);
        double da = channel.da_;
        if(fabs(da) > angles::from_degrees(90.0)) {
            continue;
        }
        double dist = channel.min_dist_;
        if(channel.length() >= distToTarget) {   //  valid
            //ROS_INFO("Valid channel found with da %f at %zu", da, channels.size() - 1);
            double score = computeChannelScore(da, dist);
            if(score > best_score) {
                best_score = score;
                best_idx = channel_idx;
                //ROS_INFO("Better channel found with da %f at %d", best_da, best_idx);
            }
        }
    }
    //if(best_idx < 0) {
    //    ROS_WARN("No valid channel found - trying channels at half distToTarget");
    //    for(unsigned int channel_idx = 0; channel_idx < channels.size(); channel_idx++) {
    //        const DriveChannel & channel = channels.at(channel_idx);
    //        double da = channel.da_;
    //        double dist = channel.min_dist_;
    //        if(channel.length() >= 0.5 * distToTarget) {   //  valid
    //            double score = computeChannelScore(da, dist);
    //            if(score > best_score) {
    //                best_score = score;
    //                best_idx = channel_idx;
    //            }
    //        }
    //    }
    //}

    return best_idx;
}

int ChannelController::evaluateSafeChannels(const std::vector<DriveChannel> & channels, bool onlyBackwards) const
{
    int best_idx = -1;
    double best_score = -1.0;
    for(unsigned int channel_idx = 0; channel_idx < channels.size(); channel_idx++) {
        const DriveChannel & channel = channels.at(channel_idx);
        double dir = channel.direction();
        double dist = channel.min_dist_;
        double length = channel.length();
        if(onlyBackwards) {
            if(fabs(dir) < angles::from_degrees(90))
                continue;
        }
        // Scoring is based on dist and length
        // Last term is added to decide between equal channels
        double score = straight_up(dist, safe_channel_width_/2.0, safe_channel_width_) +
            straight_up(length, 0.0, 2.0 * safe_channel_width_) +
            0.05 * straight_up(dist, 0.0, 10.0);
        if(score > best_score) {
            best_score = score;
            best_idx = channel_idx;
            //ROS_INFO("Better safe channel found with da %f at %d", best_da, best_idx);
        }
    }
    return best_idx;
}


bool ChannelController::computeVelocityCommands(geometry_msgs::Twist & cmd_vel)
{
    cmd_vel = geometry_msgs::Twist();   // init to 0
    ScopedVelocityStatus velStatus(cmd_vel, pub_status_marker_, pub_sound_, pub_led_, costmap_ros_, costmap_ros_->getGlobalFrameID());

    if(ros::Time::now() - last_odom_.header.stamp > ros::Duration(0.5)) {
        ROS_ERROR("Last odom is too old: %f - cannot produce commands.",
                (ros::Time::now() - last_odom_.header.stamp).toSec());
        return false;
    }
    if(!updateVoronoi()) {
        ROS_ERROR_THROTTLE(1.0, "updateVoronoi failed.");
        return false;
    }

    if(!localizeGlobalPlan(current_waypoint_)) {
        return false;
    }

    // FIXME later: we should be able to skip/advance to waypoints furhter in the plan if they are reached.
    // No need to follow the plan backwards if we somehow got ahead towards goal - only that counts.
    while(!local_plan_.empty() && currentWaypointReached()) {
        current_waypoint_++;
        last_progress_time_ = ros::Time::now();
        if(!localizeGlobalPlan(current_waypoint_)) {
            return false;
        }
    } // FIXME later: more efficient, esp for skipping: wayPOintReache(idx)

    if(local_plan_.empty()) {
        return true;    // FIXME At goal, but move_base wants us to say we did a velocity command always
    }
    tf::Stamped<tf::Pose> robot_pose;
    if(!costmap_ros_->getRobotPose(robot_pose)) {
        return false;
    }

    if(no_progress_hard_clear_time_ > 0 && 
            ros::Time::now() - last_progress_time_ > ros::Duration(no_progress_hard_clear_time_)) {
        ROS_WARN("No progress for %f s - calling /call_clear.", no_progress_hard_clear_time_);
        std_msgs::Empty e;
        pub_call_clear_.publish(e);
        last_progress_time_ = ros::Time::now(); // prevent this happening all the time, essentially reset

        kobuki_msgs::Sound snd;
        snd.value = kobuki_msgs::Sound::OFF;
        pub_sound_.publish(snd);
        return false;
    }

    // check for local wpt in obstacle: call replanning if that happens
    bool local_wpt_in_bounds;
    double dist_at_next_wpt = getDistanceAtPose(local_plan_.front(), &local_wpt_in_bounds);
    if(local_wpt_in_bounds) {   // out of bounds OK - far away - drive there first
        if(dist_at_next_wpt <= 0.0) {
            ROS_WARN_THROTTLE(1.0, "Next waypoint in obstacle - requesting new plan");
            return false;
        }
    }

    visualization_msgs::MarkerArray channelMarkers;

    tf::Pose currentTarget = local_plan_.front();

    channelMarkers.markers.push_back(createPoseMarker(currentTarget, 1.0, 1.0, 0.0, "current_target"));

    tf::Pose relativeTarget = robot_pose.inverseTimes(currentTarget);
    double distToTarget = relativeTarget.getOrigin().length();

    if(handleGoalTurn(cmd_vel, robot_pose, distToTarget, velStatus)) {
        // don't compute a channel, just turn to goal
        channelMarkers.markers.push_back(createChannelMarkers(std::vector<DriveChannel>(), 0.0, -1));
        pub_markers_.publish(channelMarkers);   // pub empty channels
        return true;
    }

    int getting_to_safe_wpt = getToSafeWaypoint(cmd_vel, robot_pose, relativeTarget, velStatus);
    if(getting_to_safe_wpt == 1) {
        return true;
    } else if(getting_to_safe_wpt == -1) {
        ROS_ERROR_THROTTLE(1.0, "getToSafeWaypoint failed.");
        velStatus.setNoSafeChannel();
        return false;
    } else if(getting_to_safe_wpt == -2) {
        ROS_INFO_THROTTLE(1.0, "getToSafeWaypoint succeeded - querying new global plan.");
        return false;
    }

    // TODO must occur everywhere, where we use this for scaling, etc.
    // check locations
    // HERE, safe waypoint, scoring
    double cur_tv = last_odom_.twist.twist.linear.x;
    double min_channel_width = safe_waypoint_channel_width_ +
        (safe_waypoint_channel_width_at_max_tv_ - safe_waypoint_channel_width_) *
        straight_up(cur_tv, min_tv_, max_tv_);
    // TODO need to be more aggressive here, otherwise never channels
    // TODO aspect ratio channels?
    std::vector<DriveChannel> channels = computeChannels(robot_pose, relativeTarget,
            min_channel_width/2.0);
    // aspect could be for socring
    // Maybe something here: best ideal chans, then go down in what we need - drive those more convervative.
    // more convervative can punish width a lot
    if(channels.size() == 0)
        channels = computeChannels(robot_pose, relativeTarget, safe_waypoint_channel_width_/2.0);
    if(channels.size() == 0) {
        ROS_WARN("No safe_waypoint_channel_width channels found - switching to CSGetToSafeWaypointDist");
        state_ = CSGetToSafeWaypointDist;
        double prob = drand48();
        if(prob < 0.35)
            safe_waypoint_state_ = SWSBestBackwardsChannelTurnIn;
        else if(prob < 0.7)
            safe_waypoint_state_ = SWSBestBackwardsChannelTurnOut;
        else
            safe_waypoint_state_ = SWSBestChannel;
        get_to_safe_waypoint_start_time_ = ros::Time::now();
        // just enable behavior here, next iteration will execute.
        return true;
    }

    int best_idx = evaluateChannels(channels, distToTarget);

    channelMarkers.markers.push_back(createChannelMarkers(channels, distToTarget, best_idx));
    pub_markers_.publish(channelMarkers);

    // no valid channel found -> replan global please
    if(best_idx < 0) {
        ROS_WARN_THROTTLE(0.5, "No valid channel found.");
        velStatus.setNoValidChannel();
        return false;
    }

    ROS_DEBUG("Best channel found with da %.0f deg width: %.1f m",
            angles::to_degrees(channels[best_idx].da_),
            2.0 * channels[best_idx].min_dist_);

    bool foundChannelCmd = computeVelocityForChannel(channels[best_idx], cmd_vel, velStatus);
    if(!foundChannelCmd) {
        ROS_ERROR("%s: Could not determine channel velocity", __func__);
    }

    ROS_DEBUG("%sSending cmd vel: %f %f", foundChannelCmd ? "" : "NOT ", cmd_vel.linear.x, cmd_vel.angular.z);

    last_cmd_vel_time_ = ros::Time::now();
    return foundChannelCmd;
}


ChannelController::ScopedVelocityStatus::ScopedVelocityStatus(geometry_msgs::Twist & cmdVel,
        ros::Publisher & pubMarkers, ros::Publisher & pubSound, ros::Publisher & pubLED,
        const costmap_2d::Costmap2DROS* costmap, std::string map_frame_id) :
    cmd_vel(cmdVel), pub_markers(pubMarkers), pub_sound(pubSound), pub_led(pubLED), costmap(costmap), map_frame_id(map_frame_id)
{
    status << std::setprecision(2) << std::fixed;
    state = CSNone;
}

ChannelController::ScopedVelocityStatus::~ScopedVelocityStatus()
{
    publishStatus();
}

void ChannelController::ScopedVelocityStatus::setAtGoalPosStopToTurn(double angle_to_goal, double cur_tv, double cur_rv)
{
    status << "At Goal Pos" << std::endl <<
        "-> Stop to turn" << std::endl <<
        "Angle: " << angles::to_degrees(angle_to_goal) << " deg" << std::endl <<
        "Cur TV: " << cur_tv << " Cur RV: " << angles::to_degrees(cur_rv) << " deg/s" << std::endl;
    state = CSGoalTurn;
}

void ChannelController::ScopedVelocityStatus::setAtGoalPosTurnToGoal(double angle_to_goal, double cur_tv, double cur_rv)
{
    status << "At Goal Pos" << std::endl <<
        "-> Turn to goal" << std::endl <<
        "Angle: " << angles::to_degrees(angle_to_goal) << std::endl <<
        "Cur TV: " << cur_tv << " Cur RV: " << angles::to_degrees(cur_rv) << " deg/s" << std::endl;
    state = CSGoalTurn;
}

void ChannelController::ScopedVelocityStatus::setChannelStopToTurn(double rel_channel_dir, double cur_tv, double cur_rv)
{
    status << "Follow Channel" << std::endl <<
        "-> Stop to turn" << std::endl <<
        "Angle: " << angles::to_degrees(rel_channel_dir) << std::endl <<
        "Cur TV: " << cur_tv << " Cur RV: " << angles::to_degrees(cur_rv) << " deg/s" << std::endl;
    state = CSFollowChannel;
}

void ChannelController::ScopedVelocityStatus::setChannelTurnToChannel(double rel_channel_dir, double cur_tv, double cur_rv)
{
    status << "Follow Channel" << std::endl <<
        "-> Turn to channel" << std::endl <<
        "Angle: " << angles::to_degrees(rel_channel_dir) << std::endl <<
        "Cur TV: " << cur_tv << " Cur RV: " << angles::to_degrees(cur_rv) << " deg/s" << std::endl;
    state = CSFollowChannel;
}

void ChannelController::ScopedVelocityStatus::setChannelFollowChannel(double rel_channel_dir,
        double cur_tv, double cur_rv,
        const std::string & tv_scale, const std::string & rv_scale)
{
    status << "Follow Channel" << std::endl <<
        "Angle: " << angles::to_degrees(rel_channel_dir) << std::endl <<
        "Cur TV: " << cur_tv << " Cur RV: " << angles::to_degrees(cur_rv) << " deg/s" << std::endl <<
        "TV limited: " << tv_scale << std::endl <<
        "RV limited: " << rv_scale << std::endl;
    state = CSFollowChannel;
}

void ChannelController::ScopedVelocityStatus::setNoValidChannel()
{
    status << "No valid channel" << std::endl;
    state = CSNone;
}

void ChannelController::ScopedVelocityStatus::setNoSafeChannel()
{
    status << "No safe channel" << std::endl;
    state = CSNone;
}

void ChannelController::ScopedVelocityStatus::setGetToSafeWaypoint(double cur_dist, double active_time, enum SafeWaypointState safe_waypoint_st)
{
    status << "Get To Safe WPT" << std::endl;
    switch(safe_waypoint_st) {
        case SWSBestChannel:
            status << "-> Best Channel" << std::endl;
            break;
        case SWSBestBackwardsChannelTurnIn:
            status << "-> Best Backwards In" << std::endl;
            break;
        case SWSBestBackwardsChannelTurnOut:
            status << "-> Best Backwards Out" << std::endl;
            break;
        default:
            status << "-> ERROR undefined: " << safe_waypoint_st << std::endl;
            break;
    }
    status << "Cur Safe Dist: " << cur_dist << std::endl <<
        "Active for: " << active_time << " s" << std::endl;
    state = CSGetToSafeWaypointDist;
    safe_waypoint_state = safe_waypoint_st;
}

void ChannelController::ScopedVelocityStatus::publishStatus()
{
    status << "TV: " << cmd_vel.linear.x << " m/s" << std::endl <<
        "RV: " << angles::to_degrees(cmd_vel.angular.z) << " deg/s" << std::endl;

    visualization_msgs::Marker statusMarker;
    statusMarker.header.stamp = ros::Time(0);
    statusMarker.header.frame_id = map_frame_id;
    statusMarker.ns = "status";
    statusMarker.id = 0;
    statusMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    statusMarker.action = visualization_msgs::Marker::ADD;
    statusMarker.text = status.str();
    statusMarker.scale.z = 0.25;

    //ROS_INFO("Status: %s", status.str().c_str());

    // put status marker near robot
    tf::Stamped<tf::Pose> robot_pose;
    if(!costmap->getRobotPose(robot_pose)) {
        statusMarker.pose.orientation.w = 1.0;
    } else {
        tf::poseTFToMsg(robot_pose, statusMarker.pose);
    }
    statusMarker.pose.position.x += 1.0;
    statusMarker.pose.position.z += 1.0;
    statusMarker.color.r = 1.0;
    statusMarker.color.g = 1.0;
    statusMarker.color.b = 0.0;
    statusMarker.color.a = 1.0;
    statusMarker.lifetime = ros::Duration(0);
    statusMarker.frame_locked = false;

    pub_markers.publish(statusMarker);

    kobuki_msgs::Led led;
    if(cmd_vel.linear.x == 0 && cmd_vel.angular.z == 0) {   // for some reason we're stopping
        led.value = kobuki_msgs::Led::BLACK;
    } else if(state == CSFollowChannel) {
        led.value = kobuki_msgs::Led::GREEN;
    } else if(state == CSGoalTurn) {
        led.value = kobuki_msgs::Led::ORANGE;
    } else if(state == CSGetToSafeWaypointDist) {
        led.value = kobuki_msgs::Led::RED;
    }
    pub_led.publish(led);

    static ros::Time last_sound_time = ros::Time(0);
    if(ros::Time::now() - last_sound_time > ros::Duration(1.0)) {
        kobuki_msgs::Sound sound;
        sound.value = 0;
        if(state == CSGetToSafeWaypointDist) {
            sound.value = kobuki_msgs::Sound::RECHARGE;
        } else if(state == CSGoalTurn) {
            sound.value = kobuki_msgs::Sound::BUTTON;
        }
        if(sound.value > 0) {
            pub_sound.publish(sound);
            last_sound_time = ros::Time::now();
        }
    }
}

}

