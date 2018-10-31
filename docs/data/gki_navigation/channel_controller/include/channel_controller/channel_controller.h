#ifndef CHANNEL_CONTROLLER_H
#define CHANNEL_CONTROLLER_H

#include <nav_core/base_local_planner.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <dynamicvoronoi/dynamicvoronoi.h>

namespace channel_controller
{
    class DriveChannel
    {
        public:
            tf::Pose from_pose_;
            tf::Pose to_pose_;
            double min_dist_;      ///< min width along the whole channel
            double da_;            ///< angle delta to next waypoint

            double length() const { return from_pose_.inverseTimes(to_pose_).getOrigin().length(); }
            double direction() const {
                tf::Pose relPose = from_pose_.inverseTimes(to_pose_);
                double channel_dir = atan2(relPose.getOrigin().y(), relPose.getOrigin().x());
                return channel_dir;
            }

    };

    class ChannelController : public nav_core::BaseLocalPlanner
    {
        public:
            enum ChannelControllerState
            {
                CSNone,
                CSFollowChannel,
                CSGoalTurn,
                CSGetToSafeWaypointDist,
            };
            enum SafeWaypointState {
                SWSNone,
                SWSBestChannel,
                SWSBestBackwardsChannelTurnIn,
                SWSBestBackwardsChannelTurnOut,
            };

            /// Simple class that reports on the current status for computeVelocityCommands.
            class ScopedVelocityStatus
            {
                public:
                    ScopedVelocityStatus(geometry_msgs::Twist & cmdVel,
                            ros::Publisher & pubStatus,
                            ros::Publisher & pubSound, ros::Publisher & pubLED,
                            const costmap_2d::Costmap2DROS* costmap,
                            std::string map_frame_id);
                    ~ScopedVelocityStatus();

                    // goal approach
                    void setAtGoalPosStopToTurn(double angle_to_goal, double cur_tv, double cur_rv);
                    void setAtGoalPosTurnToGoal(double angle_to_goal, double cur_tv, double cur_rv);

                    // default channel behaviors
                    void setChannelStopToTurn(double rel_channel_dir, double cur_tv, double cur_rv);
                    void setChannelTurnToChannel(double rel_channel_dir, double cur_tv, double cur_rv);

                    void setChannelFollowChannel(double rel_channel_dir,
                            double cur_tv, double cur_rv,
                            const std::string & tv_scale, const std::string & rv_scale);

                    // failures/recoveries
                    void setNoSafeChannel();
                    void setNoValidChannel();
                    void setGetToSafeWaypoint(double cur_dist, double active_time, enum SafeWaypointState safe_waypoint_state);

                private:
                    void publishStatus();

                private:
                    geometry_msgs::Twist & cmd_vel;
                    ros::Publisher & pub_markers;
                    ros::Publisher & pub_sound;
                    ros::Publisher & pub_led;
                    const costmap_2d::Costmap2DROS* costmap;
                    std::string map_frame_id;

                    enum ChannelControllerState state;
                    enum SafeWaypointState safe_waypoint_state;

                    std::stringstream status;
            };

        public:
            ChannelController();
            virtual ~ChannelController();

            virtual void initialize(std::string name,
                    tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);

            virtual bool isGoalReached();

            virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped> & plan);

            virtual bool computeVelocityCommands(geometry_msgs::Twist & cmd_vel);

        protected:
            bool updateVoronoi();
            void visualizeVoronoi();

            /// Transform global_plan_ to the controller frame starting at start_index.
            bool localizeGlobalPlan(unsigned int start_index);

            bool currentWaypointReached() const;

            /// Compute max length of channel in a direction angle that has guaranteed clearance_dist.
            DriveChannel computeChannel(tf::Pose from_pose, tf::Pose to_pose, double clearance_dist) const;

            std::vector<DriveChannel> computeChannels(const tf::Pose & robotPose,
                    const tf::Pose & relativeTarget, double minDist) const;

            double computeChannelScore(double da, double dist) const;

            /// Evaluate channels and return the index of the best one.
            int evaluateChannels(const std::vector<DriveChannel> & channels, double distToTarget) const;

            /// Find the safest channel independent of target.
            int evaluateSafeChannels(const std::vector<DriveChannel> & channels, bool onlyBackwards) const;

            bool computeVelocityForChannel(const DriveChannel & channel, geometry_msgs::Twist & cmd_vel, ScopedVelocityStatus & status) const;

            void computeVelocityForSafeChannel(const DriveChannel & channel, geometry_msgs::Twist & cmd_vel, ScopedVelocityStatus & status, bool turn_in) const;

            void limitTwist(geometry_msgs::Twist & cmd_vel) const;

            bool handleGoalTurn(geometry_msgs::Twist & cmd_vel,
                    const tf::Stamped<tf::Pose> & robotPose, double distToTarget,
                    ScopedVelocityStatus & status);

            double getDistanceAtPose(const tf::Pose & pose, bool* in_bounds) const;

            /// Execute getToSafeWaypoint behavior
            /**
             * \return 0 if not active, 1 if active, -1 if failed, -2 if suceeded.
             */
            int getToSafeWaypoint(geometry_msgs::Twist & cmd_vel,
                    const tf::Pose & robotPose, const tf::Pose & relativeTarget,
                    ScopedVelocityStatus & status);

            visualization_msgs::Marker createChannelMarkers(
                    const std::vector<DriveChannel> & channels, double min_good_dist,
                    int best_idx) const;

            visualization_msgs::Marker createPoseMarker(const tf::Pose & pose,
                    double r, double g, double b,
                    const std::string & ns, int id = 0) const;

            inline double sign(double x) const {
                if(x < 0)
                    return -1.0;
                return 1.0;
            }

            void odometryCallback(const nav_msgs::Odometry & odom);
            void laserCallback(const sensor_msgs::LaserScan & laser);

            double straight_up(double x, double a, double b) const;
            double straight_down(double x, double a, double b) const;

        protected:
            tf::TransformListener* tf_;
            costmap_2d::Costmap2DROS* costmap_ros_;
            costmap_2d::Costmap2D* costmap_;
            DynamicVoronoi voronoi_;

            /// Current waypoint that we are approaching in the global plan.
            unsigned int current_waypoint_;
            /// Global plan as set externally
            std::vector<geometry_msgs::PoseStamped> global_plan_;
            /// Localized global plan in the cost map's global frame
            /// starting at the current_waypoint_
            std::vector< tf::Stamped<tf::Pose> > local_plan_;

            ros::Subscriber sub_laser_;
            ros::Subscriber sub_odom_;
            ros::Publisher pub_markers_;
            ros::Publisher pub_status_marker_;
            ros::Publisher pub_local_plan_;
            ros::Publisher pub_sound_;
            ros::Publisher pub_led_;

            ros::Publisher pub_call_clear_;

            nav_msgs::Odometry last_odom_;
            sensor_msgs::LaserScan last_laser_;

            ros::Time last_cmd_vel_time_;   ///< last time we send a command

            ros::Time get_to_safe_waypoint_start_time_;

            ros::Time waiting_for_obstacles_start_time_;

            ros::Time last_progress_time_;

            ros::Time goal_turn_start_time_;

            enum ChannelControllerState state_;
            enum SafeWaypointState safe_waypoint_state_;

            // Controller Parameters
            bool use_laser_;
            bool use_costmap_;

            /// Minimum channel width that is allowed to steer to a waypoint
            /** 
             * If no safe_waypoint_channel_width channel is available 
             * the best channel with safe_channel_width will be chosen 
             * to get out of an obstacle. If none is available we'll fail
             * and fallback to recoveries.
             * If we're below safe_waypoint_channel_width and find channels with safe_channel_width
             * go there for at least min_get_to_safe_dist_time_. Keep going until 
             * max_get_to_safe_dist_time_ or until we are at safe_waypoint_channel_width.
             * If we get to max_get_to_safe_dist_time_ we need recoveries.
             */
            double safe_waypoint_channel_width_;
            double safe_waypoint_channel_width_at_max_tv_;
            /// Minimum channel width that is allowed at all
            double safe_channel_width_;

            double max_channel_length_;

            double channel_score_da_;
            double channel_score_dist_;

            double min_get_to_safe_dist_time_;
            double max_get_to_safe_dist_time_;

            /// Waypoints within this are considered reached (unless goal wpt)
            double waypoint_reached_dist_;
            double waypoint_reached_dist_at_max_tv_;
            double waypoint_reached_angle_;
            /// Goal waypoint is considered reached
            double goal_reached_dist_;
            double goal_reached_angle_;

            double min_tv_;         ///< min tv that keeps the robot moving
            double max_tv_;         ///< max tv that the robot can do
            double min_rv_;         ///< min rv that keeps rotating when also driving with tv
            double max_rv_;         ///< max rv that the robot can do
            double min_inplace_rv_; ///< min rv that makes the robot rotate when not driving with tv

            double stopped_tv_;     ///< Trans vel smaller than this - consider stopped
            double stopped_rv_;     ///< Rot vel smaller than this - consider stopped

            // FIXME later: limit this somewhere, but also consider braking for obstacles hard!
            double max_accel_tv_;   ///< Max change in tv per second
            double max_accel_rv_;   ///< Max change in rv per second

            /// Wait if there are no obstacles found at all for this time.
            /// Careful: will not move in empty areas!
            /// Set to <= 0 to disable, Set to very large to never move in perceived empty areas.
            double wait_for_obstacles_time_;

            /// If > 0 and no new waypoints reached for no_progress_hard_clear_time, call
            /// node to hard clear costmaps.
            double no_progress_hard_clear_time_;

            bool visualize_voronoi_;
            double vis_max_dist_;
    };

}

#endif

