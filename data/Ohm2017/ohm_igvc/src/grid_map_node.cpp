// ros includes

#include <ros/ros.h> 
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Quaternion.h>
#include <ohm_igvc/get_successors.h>
#include <ohm_igvc/cell_to_real.h>
#include <ohm_igvc/real_to_cell.h>
#include <ohm_igvc/pixel_locations.h>
#include <ohm_igvc/position_update.h>
#include <ohm_igvc/drive_mode.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

// opencv includes

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// stl includes

#include <string>
#include <cmath>
#include <queue>

class grid_map {
	public:
		grid_map(unsigned short threshold); // 
		void laser_scan_update(const sensor_msgs::LaserScan::ConstPtr &scan); //  
		void point_cloud_update(const sensor_msgs::PointCloud::ConstPtr &points); // 
		void camera_update(const ohm_igvc::pixel_locations::ConstPtr &cam);
		void odometry_update(const geometry_msgs::Pose2D::ConstPtr &odom); // 
		void drive_mode_callback(const ohm_igvc::drive_mode::ConstPtr &mode);
		void display_map(const ros::TimerEvent &e);
		void save() { cv::imwrite("map_1.bmp", m_world); };
	private:
		bool successors_callback(ohm_igvc::get_successors::Request &rq, ohm_igvc::get_successors::Response &rp); // 
		bool cell_to_real_callback(ohm_igvc::cell_to_real::Request &rq, ohm_igvc::cell_to_real::Response &rp); //
		bool real_to_cell_callback(ohm_igvc::real_to_cell::Request &rq, ohm_igvc::real_to_cell::Response &rp); //
		// double obstacle_cost(int x, int y); // for later
		double quaternion_to_euler(geometry_msgs::Quaternion q); //
		// double distance(Point32 first, Node second) { return std::hypot((second.x - first.x), (second.y - first.y)); };

		cv::Mat m_world;

		bool auto_mode;
		
		// position
		geometry_msgs::Pose2D odometry;

		// map size
		double m_height, m_width, m_resolution;
		int m_grid_x, m_grid_y;
		geometry_msgs::Point raster_reference;

		// occupancy
		int m_threshold;

		ros::Subscriber camera_input, laser_input, odometry_input, robot_mode;
		ros::Publisher robot_cell_output;
		ros::ServiceServer successors, cell_to_real, real_to_cell;
		ros::NodeHandle node;
		ros::Timer map_display_timer;	
		tf::TransformListener listener;	
};


// members
/*
	// position
	double m_theta;
	geometry_msgs::Pose odometry;

	// map size
	const double m_height, m_width, m_resolution;
	const int m_grid_x, m_grid_y;

	// ros
	ros::Subscriber data_input_1, data_input_2, odometry_input;
	ros::NodeHandle node;
	ros::Timer map_display_timer;

*/	

grid_map::grid_map(unsigned short threshold = 200) :
	m_width(100),
	m_height(100),
	m_resolution(0.2),
	m_threshold(threshold)
 {
	auto_mode = false;
	bool show_map = false, use_pcl = true;
	std::string odom_topic = "/ohm/odom", laser_topic = "/ohm/laser", camera_topic = "/ohm/eyes";
	raster_reference.x = 0.0; raster_reference.y = 0.0;
	
	// parameters
	node.param("show_map", show_map, show_map);
	node.param("map_odometry_topic", odom_topic, odom_topic);
	node.param("map_use_pcl", use_pcl, use_pcl);
	node.param("map_laser_topic", laser_topic, laser_topic);
	node.param("map_camera_topic", camera_topic, camera_topic);
	node.param("map_raster_x", raster_reference.x, raster_reference.x);
	node.param("map_raster_y", raster_reference.y, raster_reference.y);
	node.param("map_threshold", m_threshold, m_threshold);
	node.param("map_resolution", m_resolution, m_resolution);
	node.param("map_width", m_width, m_width);
	node.param("map_height", m_height, m_height);

	m_grid_x = (m_width / m_resolution);
	m_grid_y = (m_height / m_resolution);

	camera_input = node.subscribe<ohm_igvc::pixel_locations>(camera_topic, 3, &grid_map::camera_update, this);

	if(use_pcl) laser_input = node.subscribe<sensor_msgs::PointCloud>(laser_topic, 3, &grid_map::point_cloud_update, this);
	else laser_input = node.subscribe<sensor_msgs::LaserScan>(laser_topic, 3, &grid_map::laser_scan_update, this);
	odometry_input = node.subscribe<geometry_msgs::Pose2D>(odom_topic, 3, &grid_map::odometry_update, this);
	robot_mode = node.subscribe<ohm_igvc::drive_mode>("drive_mode", 1, &grid_map::drive_mode_callback, this);

	robot_cell_output = node.advertise<ohm_igvc::position_update>("/ohm/robot_cell_position", 5);

	successors = node.advertiseService("get_successors", &grid_map::successors_callback, this);
	cell_to_real = node.advertiseService("cell_to_real", &grid_map::cell_to_real_callback, this);
	real_to_cell = node.advertiseService("real_to_cell", &grid_map::real_to_cell_callback, this);

	if(show_map) {
		map_display_timer = node.createTimer(ros::Duration(0.5), &grid_map::display_map, this);
		cv::namedWindow("Map", cv::WINDOW_AUTOSIZE);
	}

	m_world.create(m_grid_y, m_grid_x, CV_16U);
}

void grid_map::laser_scan_update(const sensor_msgs::LaserScan::ConstPtr &scan) {
	if(!auto_mode) return;
	for(int point = 0; point < scan->ranges.size(); point++) {
		if(scan->ranges[point] < scan->range_max && scan->ranges[point] > scan->range_min) {
			double ax = (scan->ranges[point] * std::sin((scan->angle_increment * point) + odometry.theta)) + odometry.x;
			double ay = (scan->ranges[point] * std::cos((scan->angle_increment * point) + odometry.theta)) + odometry.y;

			int i = (ax - raster_reference.x) / m_resolution;
			int j = (ay + raster_reference.y) / m_resolution;

			// ROS_INFO("Point at (%d, %d)", i, j);

			if(i > m_world.cols || i < 0) continue;
			if(j > m_world.rows || j < 0) continue;
			
			if(m_world.ptr<short>(j)[i] < m_threshold) {
				ROS_INFO("incrementing (%d, %d)", i, j);	
				m_world.ptr<short>(j)[i]++;
			} else {
				ROS_INFO("cell (%d, %d) has saturated to threshold %d", i, j, m_threshold);
			}
		}
	}
}

void grid_map::point_cloud_update(const sensor_msgs::PointCloud::ConstPtr &points) {
	if(!auto_mode) return;
	for(int point = 0; point < points->points.size(); point++) {
		/*
		double cos_heading = std::cos(odometry.theta), sin_heading = std::sin(odometry.theta);
		float x_prime = points->points[point].x, y_prime = points->points[point].y;

		int i = ((odometry.x + ((x_prime * cos_heading) - (y_prime * sin_heading))) - raster_reference.x) / m_resolution;
		int j = ((odometry.y + ((y_prime * cos_heading) + (x_prime * sin_heading))) + raster_reference.y) / m_resolution;
		*/

		int i = points->points[point].x / m_resolution;
		int j = points->points[point].y / m_resolution;
		
		if(i > m_world.cols || i < 0) continue;
		if(j > m_world.rows || j < 0) continue;
			
		if(m_world.ptr<short>(j)[i] < m_threshold) m_world.ptr<short>(j)[i]++;
	}
}

void grid_map::camera_update(const ohm_igvc::pixel_locations::ConstPtr &cam) {
	if(!auto_mode) return;
	tf::StampedTransform laser, robot;

	try {
		listener.lookupTransform("ohm_base_link", "world", cam->time.data, robot);
		listener.lookupTransform("sick_laser_link", "ohm_base_link", cam->time.data, laser);
	} catch(tf::TransformException e) {
		ROS_ERROR("Dropped camera frame: %s", e.what());
		return;
	}

	double roll, pitch, yaw;

	tf::Matrix3x3(robot.getRotation()).getRPY(roll, pitch, yaw);

	double cos_heading = std::cos(yaw), sin_heading = std::sin(yaw);
	double odom_origin_x = robot.getOrigin().x() + laser.getOrigin().x(), odom_origin_y = robot.getOrigin().y() + laser.getOrigin().y();

	for(int pixel = 0; pixel < cam->pixelLocations.size(); pixel++) {
		float x_prime = cam->pixelLocations[pixel].x, y_prime = cam->pixelLocations[pixel].y;

		int i = (odom_origin_x + ((x_prime * cos_heading) - (y_prime * sin_heading)) - raster_reference.x) / m_resolution;
		int j = (odom_origin_y + ((y_prime * cos_heading) + (x_prime * sin_heading)) + raster_reference.y) / m_resolution;

		if(i > m_world.cols || i < 0) continue;
		if(j > m_world.rows || j < 0) continue;
			
		if(m_world.ptr<short>(j)[i] < m_threshold) m_world.ptr<short>(j)[i]++;
	}
}

void grid_map::odometry_update(const geometry_msgs::Pose2D::ConstPtr &odom) {
	odometry = *odom;

	ohm_igvc::position_update p;
	p.x = std::trunc((odometry.x - raster_reference.x) / m_resolution);
	p.y = std::trunc((odometry.y + raster_reference.y) / m_resolution);
	p.real.x = odometry.x;
	p.real.y = odometry.y;

	robot_cell_output.publish(p);
}

void grid_map::display_map(const ros::TimerEvent &e) {
	cv::imshow("Map", m_world);
	cv::waitKey(10);
}

bool grid_map::cell_to_real_callback(ohm_igvc::cell_to_real::Request &rq, ohm_igvc::cell_to_real::Response &rp) {
	if((rq.x < 0 || rq.x > m_world.cols) || (rq.y < 0 || rq.y > m_world.rows)) return false;
	rp.real_coordinate.x = ((rq.x * m_resolution) - (m_resolution / 2)) + raster_reference.x;
	rp.real_coordinate.y = ((rq.y * m_resolution) - (m_resolution / 2)) - raster_reference.y;
	return true;
}

bool grid_map::real_to_cell_callback(ohm_igvc::real_to_cell::Request &rq, ohm_igvc::real_to_cell::Response &rp) {
	rp.x = (rq.real_coordinate.x - raster_reference.x) / m_resolution; 
	rp.y = (rq.real_coordinate.y + raster_reference.y) / m_resolution;

	if((rp.x < 0 || rp.x > m_world.cols) || (rp.y < 0 || rp.y > m_world.rows)) return false;
	return true;
}

bool grid_map::successors_callback(ohm_igvc::get_successors::Request &rq, ohm_igvc::get_successors::Response &rp) {
	ROS_INFO("Successors request for (%d, %d)", rq.x, rq.y);
	ROS_INFO("Map size: (%d, %d)", m_world.cols, m_world.rows);
	if((rq.x < 0 || rq.x > m_world.cols) || (rq.y < 0 || rq.y > m_world.rows)) return false;
	
	short *above = nullptr, *on = m_world.ptr<short>(rq.y), *below = nullptr;
	if(rq.y > 0) above = m_world.ptr<short>(rq.y - 1);
	if(rq.y < m_world.rows - 1) below = m_world.ptr<short>(rq.y + 1);

	if(!above) {
		for(int i = 0; i < 3; i++) {
			rp.nodes[i].t_cost = -1;
			rp.nodes[i].x = -1;
			rp.nodes[i].y = -1;
		}
	} else {
		for(int i = -1; i < 2; i++) {
			rp.nodes[i + 1].t_cost = above[rq.x + i]; // obstacle_cost((rq.x + i), rq.y - 1);
			rp.nodes[i + 1].x = rq.x + i;
			rp.nodes[i + 1].y = rq.y - 1;
		}
	}

	rp.nodes[3].t_cost = on[rq.x + 1]; // obstacle_cost((rq.x + 1), rq.y);
	rp.nodes[3].x = rq.x + 1;
	rp.nodes[3].y = rq.y;

	if(!below) {
		for(int i = 4; i < 7; i++) {
			rp.nodes[i].t_cost = -1;
			rp.nodes[i].x = -1;
			rp.nodes[i].y = -1;
		}
	} else {
		for(int i = -1; i < 2; i++) {
			rp.nodes[i + 5].t_cost = below[rq.x - i]; // obstacle_cost((rq.x - i), rq.y + 1);
			rp.nodes[i + 5].x = rq.x - i;
			rp.nodes[i + 5].y = rq.y + 1;
		}
	}

	rp.nodes[7].t_cost = on[rq.x - 1]; // obstacle_cost((rq.x - 1), rq.y);
	rp.nodes[7].x = rq.x - 1;
	rp.nodes[7].y = rq.y;
	
	return true;	
}

double grid_map::quaternion_to_euler(geometry_msgs::Quaternion q) {
	//	ROS_INFO("Euler Convesion"); Shamelessly taken from wikipedia
	double ysqr = q.y * q.y;

	// roll (x-axis rotation)
	double t0 = +2.0f * (q.w * q.x + q.y * q.z);
	double t1 = +1.0f - 2.0f * (q.x * q.x + ysqr);
	return std::atan2(t0, t1);
}

void grid_map::drive_mode_callback(const ohm_igvc::drive_mode::ConstPtr &mode) {
	if(mode->mode == "auto") auto_mode = true;
	else auto_mode = false;
}

/* ------------------// MAIN STUFF //------------------- */

int main(int argc, char **argv) {
	ros::init(argc, argv, "grid_map");
	grid_map world;

	ros::spin();

	world.save();

	return 0;
}
