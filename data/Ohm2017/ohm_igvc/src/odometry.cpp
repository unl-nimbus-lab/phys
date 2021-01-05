#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>
#include <ohm_igvc/coordinate_convert.h>
#include <vn300/Pose.h>
#include <ohm_igvc/target.h>
#include <string>
#include <cmath>

#define DEG2RAD(x) ((3.14159265359 * x) / 180.00)

class odometry {
  public:
    odometry();
    void position_callback(const vn300::Pose::ConstPtr &pos);
    bool convert_callback(ohm_igvc::coordinate_convert::Request &rq, ohm_igvc::coordinate_convert::Response &rp);
	double gps_x(double lon) { 
		//ROS_INFO("K_EW = %f", K_EW);
		//ROS_INFO("lon = %f", lon);
		//ROS_INFO("start lon = %f", origin.longitude);
		return (K_EW * (lon - origin.longitude)); };
	double gps_y(double lat) { 
		//ROS_INFO("K_NS = %f", K_NS);
		//ROS_INFO("lat = %f", lat);
		//ROS_INFO("start lat = %f", origin.latitude);
		return (K_NS * (lat - origin.latitude)); };

  private:
    ros::Subscriber positionSub;
    ros::Publisher pose;
    ros::ServiceServer coord_convert;
    ros::NodeHandle node;
    ohm_igvc::target origin;

    double K_NS, K_EW;

    tf::TransformBroadcaster base_br;
    tf::Transform t;
    tf::Quaternion q;

    geometry_msgs::Pose2D position;
};

odometry::odometry() {
    std::string gps_position = "/vn300/position";
	K_NS = 111120.00;

	node.param("K_NS", K_NS, K_NS);
    node.param("origin_latitude", origin.latitude, 0.0);
    node.param("origin_longitude", origin.longitude, 0.0);
    node.param("gps_position", gps_position, gps_position);

	K_EW = K_NS * std::cos(DEG2RAD(origin.latitude));

	ROS_INFO("K_NS = %f", K_NS);
	ROS_INFO("K_EW = %f", K_EW);

    positionSub = node.subscribe<vn300::Pose>(gps_position, 5, &odometry::position_callback, this);

	coord_convert = node.advertiseService("coordinate_convert", &odometry::convert_callback, this);

    pose = node.advertise<geometry_msgs::Pose2D>("/ohm/odom", 5);
    position.x = 0.0;
    position.y = 0.0;
    position.theta = 0.0;
}

void odometry::position_callback(const vn300::Pose::ConstPtr &pos) {
    position.x = gps_x(pos->position[1]);
	position.y = gps_y(pos->position[0]);

    position.theta = pos->heading[0];

    t.setOrigin(tf::Vector3(position.x, position.y, 0.0));
    q.setRPY(0, 0, position.theta);
    t.setRotation(q);

    base_br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "world", "ohm_base_link"));

	pose.publish(position);
}

bool odometry::convert_callback(ohm_igvc::coordinate_convert::Request &rq, ohm_igvc::coordinate_convert::Response &rp) {
	rp.coordinate.x = gps_x(rq.coordinate.longitude);
	rp.coordinate.y = gps_y(rq.coordinate.latitude);

	return true;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "odometry");

    odometry node;

    ros::spin();

    return 0;
}
