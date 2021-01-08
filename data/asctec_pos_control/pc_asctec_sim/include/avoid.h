#ifndef AVOID_H
#define AVOID_H

#include <ros/ros.h>
#include <atraj.h>

#include <Eigen/Dense>

#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <pc_asctec_sim/pc_traj_cmd.h>
#include <pc_asctec_sim/pc_state.h>

#include <sensor_msgs/Joy.h>
#include <visualization_msgs/Marker.h>

#include <math.h>
#include <string.h>

#define obsNumb 10

#define repel_pts 3.0	//always integer
#define repel 0.6       //radius
#define repel_t 5.0
#define repel_rings 5
#define end_off 0.5

#define real_r  0.14    //11 inches diameter
#define quad_r 0.27     //21 inches diameter

#define freq 10Trajectory_Accel * path_gen
#define maxV 0.3
#define maxA maxV

/* --------------- Data Structure Definitions ------------ */

struct obs {
	string frame;
	float x;
	float y;
	float r;
	obs * next;

}OBS;

struct Isec {

	double t;
	int spline;
	Isection * next;
	
}ISECTION;

/* -------------------- Class Definition ---------------- */
class Avoid
{
	world
	struct obs * obs_ptr;
	struct Isec * Isec_ptr;
	Trajectory_Accel * path_gen;
	pc_asctec_sim::CMatrix C;

	public:
		Avoid(std::string world_);
		~Avoid();

		void addNewINode(double t, int spline);
		void addNewONode(std::string frame, float x, float y, float r);
	private:
		void getRoots(struct obs * ob_ptr, struct Isec * I_ptr, pc_asctec_sim::pc_traj_cmd * path)
}
#endif
