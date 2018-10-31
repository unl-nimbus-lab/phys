#ifndef ASCTECQUAD_H
#define ASCTECQUAD_H

#include "asctec_control.h"
#include "atraj.h"

/* --------------- Data Structure Definitions ------------ */
typedef struct TRAIL
{
	float x,y,z, yaw;
}trail;

typedef struct QUAD_OUT
{
	bool isComplete, startTimer, goalUpdated;
	float time;
	pc_asctec_sim::SICmd TRPYcmd;
	pc_asctec_sim::pc_state state;
	TRAIL goal;

}quad_out;

typedef struct QUAD_CMD
{
	int xyFree;
	bool start, stopTimer, newPath;
	float battery;
	float x,y,vx,vy;

	path_type type;
	pc_asctec_sim::pc_traj_cmd f_path;
	geometry_msgs::Twist xyCmd;
	K_DATA kvals;
	GOAL_DATA gNew;

}quad_cmd;

/* -------------------- Class Definition ---------------- */
class AscTec_Quad
{
	public:
		AscTec_Quad(string qframe, string wframe, tf::TransformListener *listener, float dt, struct K_DATA * kvals);
		QUAD_OUT * runQuad(QUAD_CMD * cmd);
		bool setQuadGoal(struct GOAL_DATA * g);

		AscTec_Controller controller;
		Trajectory_Accel accTraj;
		string qframe_, wframe_;
	private:

		PUB_DATA *P_ptr;
		WAYPOINT *W_ptr;
		QUAD_OUT *Q_ptr;
		GOAL_DATA goal_;
		tf::TransformListener *listener_;
};

#endif
