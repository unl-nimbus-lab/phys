#ifndef AR_JACO_INTERFACE_H
#define AR_JACO_INTERFACE_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Dense>
#include <Eigen/SVD>

typedef Eigen::Matrix<float,6,6> Matrix6f;
typedef Eigen::Matrix<float,3,6> Matrix36f;
typedef Eigen::Matrix<float,6,3> Matrix63f;
typedef Eigen::Matrix<float,6,1> Vector6f;

class ArJacoInterface
{
	public:
			ArJacoInterface(void);
			~ArJacoInterface(void);
			
			void arMarkerCallback(const ar_track_alvar_msgs::AlvarMarkers& ar_marker);
			void armPosCallback(const geometry_msgs::PoseStampedConstPtr& arm_pos);
			void odomCallback(const nav_msgs::OdometryConstPtr& odom);
			void KFUpdate(void);
			void KFCorrect(bool tag_avail);
			
			geometry_msgs::TwistStamped velocitySaturate(geometry_msgs::TwistStamped vel);
	
	private:
	
			ros::NodeHandle nh_;
			
			ros::Subscriber ar_marker_sub_, arm_pos_sub_, odom_sub_;
			ros::Publisher cartesian_cmd_pub_;
			
			void watchdog(const ros::TimerEvent&);
			
			int target_marker_;
			double velocity_limit_, ang_limit_,  vel_gain_;
			double offset_x_, offset_y_, offset_z_;
			geometry_msgs::Pose arm_pos_;
            double husky_omega_, husky_vel_;
			
			ros::Timer watchdog_timer;
			double watchdog_interval_seconds;
			bool tag_avail, arm_avail, init;
			
			// state matriix
			Matrix6f A, P, Q;
            Matrix36f C;
            Eigen::Matrix3f R;
			Eigen::Vector3f z, e;
            Vector6f x, B;

			
};

int main(int argc, char **argv);

#endif //AR_JACO_INTERFACE_H
			
			
