#ifndef ATRAJ_H
#define ATRAJ_H

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>

#include <pc_asctec_sim/pc_goal_cmd.h>
#include <pc_asctec_sim/pc_feedback.h>
#include <pc_asctec_sim/pc_traj_cmd.h>
#include <pc_asctec_sim/pc_state.h>
#include <pc_asctec_sim/CMatrix.h>

#include <visualization_msgs/Marker.h>

#include <math.h>
#include <string.h>

#include <iostream>
#include <fstream>
#include <Eigen/Dense>

#define BUF 3

using namespace std;
using Eigen::MatrixXd;


/* -------------------- Enum Definitions ---------------- */

enum set_result
{
success,
stillrunning,
bufferadded,
bufferfull
};

enum path_type
{
newonly,
overwrite,
buffer
};

/* -------------------- Struct Definitions ---------------- */
typedef struct CONSTANTS
{
	float C5[24];
	float C4[24];
	float C3[24];
	float C2[24];
	float C1[24];
	float C0[24];

}constants;

typedef struct TRPY_CONSTANTS
{
	struct CONSTANTS X,Y,Z,YAW;
	int points;

}trpy_constants;

typedef struct NEW_PATH
{
	path_type type;
	pc_asctec_sim::pc_state state;
	pc_asctec_sim::pc_traj_cmd cmd;

}new_path;

typedef struct WAYPOINT
{
	bool isValid;
	pc_asctec_sim::pc_goal_cmd goal;

}waypoint;

typedef struct CMD_BUF
{
	pc_asctec_sim::pc_traj_cmd cmd[BUF];
	bool isValid[BUF];
	int buf_now;

}cmd_buf;

/* -------------------- Class Definition ---------------- */
class Trajectory_Accel
{
	public:
		Trajectory_Accel(float d_t);
		~Trajectory_Accel();

		bool getComplete();			
		void setStarted(bool val);

		struct WAYPOINT * updateWaypoint(struct WAYPOINT * goal);
		struct TRPY_CONSTANTS * getXMatrix(struct TRPY_CONSTANTS * matrix);
		enum set_result setBMatrix(struct NEW_PATH * path);
		pc_asctec_sim::CMatrix * getPathConstants(struct NEW_PATH * path, pc_asctec_sim::CMatrix * data);

	private:
		void initMembers();
		void initAMatrix();
		void initBMatrix();

		void calcAMatrix(float time);
		void solveXMatrix();

		bool setBufferedPath();
		bool isBufferFull();	
		int nextBuffer();
		int nextEmptyBuffer();

		MatrixXd A, B, X, T;	//6x6, 24x24, 24x24, 2x24
		struct CMD_BUF B_buffer;

		float dt;
		double c_time;
		ros::Time t0;
		int point;
		int points;
		bool isComplete;
		bool isDelayed;
		bool isStarted;
};
#endif
