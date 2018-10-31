#include <atraj.h>

Trajectory_Accel::Trajectory_Accel(float d_t)
{
 /* Trajectory class constructor
  * - Holds A, B, X, T matricies
  * - Can return direct X matrix constants
  * - Can return next waypoint upon call
  */

	A.resize(6,6);
	B.resize(24,24);
	X.resize(24,24);
	T.resize(2,24);

	initAMatrix();
	initBMatrix();
	initMembers();

	dt = d_t;
}

Trajectory_Accel::~Trajectory_Accel() {}

bool Trajectory_Accel::getComplete()
{
	return isComplete;
}

struct WAYPOINT * Trajectory_Accel::updateWaypoint(struct WAYPOINT * goal)
{
 /* Update next goal point in path
  * - Sets goal based on currently running trajectory
  */

	if(point == points && !isComplete) {
		if(!setBufferedPath()) {
			goal->isValid = false;
		}
	}

	if(!isComplete) {
		goal->isValid = true;
		pc_asctec_sim::pc_goal_cmd temp;
		goal->goal = temp;

		if(!isDelayed) {
			c_time += dt;
		}
		goal->goal.goal_id = c_time;

		for(int i=5; i>=0; i--) {
			goal->goal.x += X(5-i,point)*pow(c_time,i);
			goal->goal.vx += i*X(5-i,point)*pow(c_time,i-1);
			goal->goal.ax += i*(i-1)*X(5-i,point)*pow(c_time,i-2);

			goal->goal.y += X(11-i,point)*pow(c_time,i);
			goal->goal.vy += i*X(11-i,point)*pow(c_time,i-1);
			goal->goal.ay += i*(i-1)*X(11-i,point)*pow(c_time,i-2);

			goal->goal.z += X(17-i,point)*pow(c_time,i);
			goal->goal.vz += i*X(17-i,point)*pow(c_time,i-1);
			goal->goal.az += i*(i-1)*X(17-i,point)*pow(c_time,i-2);

			goal->goal.yaw += X(23-i,point)*pow(c_time,i);
			goal->goal.vyaw += i*X(23-i,point)*pow(c_time,i-1);
			goal->goal.ayaw += i*(i-1)*X(23-i,point)*pow(c_time,i-2);
		}

		if((c_time + 0.001) >= T(1,point)) {
			if(T(0,point) != 0.0 && !isDelayed) {
				isDelayed = true;
				t0 = ros::Time::now();

			}else if(isDelayed && ((ros::Time::now().toSec() - t0.toSec()) >= T(0,point))) {
				point++;
				c_time = 0.0;
				isDelayed = false;

			}else if(T(0,point) == 0.0){
				point++;
				c_time = 0.0;
			}
		}
	}else {
		goal->isValid = false;
	}

	return goal;
}

struct TRPY_CONSTANTS * Trajectory_Accel::getXMatrix(struct TRPY_CONSTANTS * matrix)
{
	matrix->points = points;
	for(int i=0; i<points; i++) {
		matrix->X.C5[i] = X(0,i);
		matrix->X.C4[i] = X(1,i);
		matrix->X.C3[i] = X(2,i);
		matrix->X.C2[i] = X(3,i);
		matrix->X.C1[i] = X(4,i);
		matrix->X.C0[i] = X(5,i);

		matrix->Y.C5[i] = X(6,i);
		matrix->Y.C4[i] = X(7,i);
		matrix->Y.C3[i] = X(8,i);
		matrix->Y.C2[i] = X(9,i);
		matrix->Y.C1[i] = X(10,i);
		matrix->Y.C0[i] = X(11,i);

		matrix->Z.C5[i] = X(12,i);
		matrix->Z.C4[i] = X(13,i);
		matrix->Z.C3[i] = X(14,i);
		matrix->Z.C2[i] = X(15,i);
		matrix->Z.C1[i] = X(16,i);
		matrix->Z.C0[i] = X(17,i);

		matrix->YAW.C5[i] = X(18,i);
		matrix->YAW.C4[i] = X(19,i);
		matrix->YAW.C3[i] = X(20,i);
		matrix->YAW.C2[i] = X(21,i);
		matrix->YAW.C1[i] = X(22,i);
		matrix->YAW.C0[i] = X(23,i);
	}
	return matrix;
}

void Trajectory_Accel::initMembers()
{
 /* Setup class member variables
  */

	c_time = 0.0;
	point = 0;
	points = 0;

	isComplete = true;
	isDelayed = false;
}

void Trajectory_Accel::initAMatrix()
{
 /* Setup A Matrix constants
  * - Called only once in constructor
  */

	for(int i = 0; i < 6; i++) {
		for(int k = 0; k < 6; k++) {
			A(i,k) = 0.0;
		}
	}

	A(0,5) = 1.0;
	A(2,4) = 1.0;
	A(4,3) = 2.0; 
}

void Trajectory_Accel::initBMatrix()
{
 /* Setup B Matrix buffer
  * - Start B Matrix current val
  * - Called only once in constructor
  */

	B_buffer.buf_now = 0;
	for(int i=0; i<BUF; i++) {
		B_buffer.isValid[i] = false;
	}
}

void Trajectory_Accel::calcAMatrix(float time)
{
 /* Set A matrix values
  * - Set matrix positions based on spline duration
  */

	for(int i=5; i>=0; i--) {
		A(1,5-i) = pow(time,i);
		A(3,5-i) = i*pow(time,i-1);
		A(5,5-i) = i*(i-1)*pow(time,i-2);
	}
}

pc_asctec_sim::CMatrix * Trajectory_Accel::getPathConstants(struct NEW_PATH *path, pc_asctec_sim::CMatrix *data)
{
	data->points = path->cmd.points;
	Eigen::MatrixXd B2 = Eigen::MatrixXd::Constant(24,data->points,0);
	Eigen::MatrixXd X2 = Eigen::MatrixXd::Constant(24,data->points,0);
	Eigen::MatrixXd T2 = Eigen::MatrixXd::Constant(2,data->points,0);

	for(int i = 0; i < path->cmd.points; i++) {
		B2(1,i) = path->cmd.x[i];
		B2(3,i) = path->cmd.vx[i];
		B2(5,i) = path->cmd.ax[i];
	
		B2(7,i) = path->cmd.y[i];
		B2(9,i) = path->cmd.vy[i];
		B2(11,i) = path->cmd.ay[i];

		B2(13,i) = path->cmd.z[i];
		B2(15,i) = path->cmd.vz[i];
		B2(17,i) = path->cmd.az[i];

		B2(19,i) = path->cmd.yaw[i];
		B2(21,i) = path->cmd.vyaw[i];
		B2(23,i) = path->cmd.ayaw[i];

		T2(0,i) = path->cmd.wait_time[i];
		T2(1,i) = path->cmd.duration[i];

	}
	B2(0,0) = path->state.x;
	B2(2,0) = path->state.vx;
	B2(4,0) = path->state.ax;

	B2(6,0) = path->state.y;
	B2(8,0) = path->state.vy;
	B2(10,0) = path->state.ay;

	B2(12,0) = path->state.z;
	B2(14,0) = path->state.vz;
	B2(16,0) = path->state.az;
	
	B2(18,0) = path->state.yaw;
	B2(20,0) = path->state.vyaw;
	B2(22,0) = path->state.ayaw;

	for(int i = 1; i < data->points; i++) {
		for(int j = 0; j < 4; j++) {
			B2(6*j,i) = B2(6*j+1,i-1);
			B2(6*j+2,i) = B2(6*j+3,i-1);
			B2(6*j+4,i) = B2(6*j+5,i-1);      
		}
	} 	

	for(int i = 0; i < data->points; i++) {
		calcAMatrix(T2(1,i));
		for(int k = 0; k < 4; k++) {
			X2.block<6,1>(6*k,i) = A.colPivHouseholderQr().solve(B2.block<6,1>(6*k,i));
		}
		data->c5x[i] = X2(0,i);
		data->c4x[i] = X2(1,i);
		data->c3x[i] = X2(2,i);
		data->c2x[i] = X2(3,i);
		data->c1x[i] = X2(4,i);
		data->c0x[i] = X2(5,i);

		data->c5y[i] = X2(6,i);
		data->c4y[i] = X2(7,i);
		data->c3y[i] = X2(8,i);
		data->c2y[i] = X2(9,i);
		data->c1y[i] = X2(10,i);
		data->c0y[i] = X2(11,i);

		data->c5z[i] = X2(12,i);
		data->c4z[i] = X2(13,i);
		data->c3z[i] = X2(14,i);
		data->c2z[i] = X2(15,i);
		data->c1z[i] = X2(16,i);
		data->c0z[i] = X2(17,i);

		data->c5yaw[i] = X2(18,i);
		data->c4yaw[i] = X2(19,i);
		data->c3yaw[i] = X2(20,i);
		data->c2yaw[i] = X2(21,i);
		data->c1yaw[i] = X2(22,i);
		data->c0yaw[i] = X2(23,i);
	}
	return data;
}

void Trajectory_Accel::solveXMatrix()
{
 /* Solves inverse matrix for X
  * - Uses eigen/dense library
  * - Prints out calculation time
  */

	ros::Time t_s = ros::Time::now();
	for(int i = 0; i < points; i++) {
		calcAMatrix(T(1,i));
		for(int k = 0; k < 4; k++) {
			X.block<6,1>(6*k,i) = A.colPivHouseholderQr().solve(B.block<6,1>(6*k,i));
		}
	}
	ros::Time t_e = ros::Time::now();
	float t_f = ((t_e - t_s).toNSec());
	ROS_INFO("Trajectory calculation took %f ms", t_f/1000000);

}

enum set_result Trajectory_Accel::setBMatrix(struct NEW_PATH * path)
{
 /* Sets the BMatrix or BMatrix buffer based on command type
  * - newonly writes if last trajectory is completed
  * - overwrite writes over existing trajectory
  * - buffer writes to next open buffer space
  */

	if((path->type == overwrite) || (path->type == newonly && isComplete == true) || (path->type == buffer && nextBuffer() == -1 && isComplete)) {
		points = path->cmd.points;
		X.resize(24, points);
		for(int i = 0; i < points; i++) {
			B(1,i) = path->cmd.x[i];
			B(3,i) = path->cmd.vx[i];
			B(5,i) = path->cmd.ax[i];
		
			B(7,i) = path->cmd.y[i];
			B(9,i) = path->cmd.vy[i];
			B(11,i) = path->cmd.ay[i];

			B(13,i) = path->cmd.z[i];
			B(15,i) = path->cmd.vz[i];
			B(17,i) = path->cmd.az[i];

			B(19,i) = path->cmd.yaw[i];
			B(21,i) = path->cmd.vyaw[i];
			B(23,i) = path->cmd.ayaw[i];

			T(0,i) = path->cmd.wait_time[i];
			T(1,i) = path->cmd.duration[i];
	
		}
		B(0,0) = path->state.x;
		B(2,0) = path->state.vx;
		B(4,0) = path->state.ax;
	
		B(6,0) = path->state.y;
		B(8,0) = path->state.vy;
		B(10,0) = path->state.ay;
	
		B(12,0) = path->state.z;
		B(14,0) = path->state.vz;
		B(16,0) = path->state.az;
		
		B(18,0) = path->state.yaw;
		B(20,0) = path->state.vyaw;
		B(22,0) = path->state.ayaw;
	
		for(int i = 1; i < points; i++) {
			for(int j = 0; j < 4; j++) {
				B(6*j,i) = B(6*j+1,i-1);
				B(6*j+2,i) = B(6*j+3,i-1);
				B(6*j+4,i) = B(6*j+5,i-1);      
			}
		} 	

		solveXMatrix();

		point = 0;
		c_time = 0.0;
		isComplete = false;		

		if(path->type == overwrite && !isComplete) {
			ROS_INFO("Last trajectory overwritten!");

		}

		return success;

	}else if(path->type == newonly && isComplete == false) {
		ROS_INFO("Trajectory rejected, trajectory already running!");
		return stillrunning;

	}else if(path->type == buffer && !isBufferFull()) {
		ROS_INFO("Trajectory added to buffer");
		B_buffer.cmd[nextEmptyBuffer()] = path->cmd;
		B_buffer.isValid[nextEmptyBuffer()] = true;
		return bufferadded;
		
	}else if(path->type == buffer && isBufferFull()) {
		ROS_INFO("Buffer full!");
		return bufferfull;
	}
}

bool Trajectory_Accel::setBufferedPath()
{
 /* Sets the BMatrix to the next buffered trajectory
  * - Automatically recalculates A and X matricies
  */
	if(nextBuffer() == -1) {
		isComplete = true;
		return false;

	}else {
		B(0,0) = B(1,points);
		B(2,0) = B(3,points);
		B(4,0) = B(5,points);
	
		B(6,0) = B(7,points);
		B(8,0) = B(9,points);
		B(10,0) = B(11,points);
	
		B(12,0) = B(13,points);
		B(14,0) = B(15,points);
		B(16,0) = B(17,points);
		
		B(18,0) = B(19,points);
		B(20,0) = B(21,points);
		B(22,0) = B(23,points);

		int nextCmd = nextBuffer();
		B_buffer.buf_now = nextCmd;
		B_buffer.isValid[nextCmd] = false;
		points = B_buffer.cmd[nextCmd].points;
		for(int i = 0; i < points; i++) {
			B(1,i) = B_buffer.cmd[nextCmd].x[i];	
			B(3,i) = B_buffer.cmd[nextCmd].vx[i];
			B(5,i) = B_buffer.cmd[nextCmd].ax[i];
		
			B(7,i) = B_buffer.cmd[nextCmd].y[i];
			B(9,i) = B_buffer.cmd[nextCmd].vy[i];
			B(11,i) = B_buffer.cmd[nextCmd].ay[i];

			B(13,i) = B_buffer.cmd[nextCmd].z[i];
			B(15,i) = B_buffer.cmd[nextCmd].vz[i];
			B(17,i) = B_buffer.cmd[nextCmd].az[i];

			B(19,i) = B_buffer.cmd[nextCmd].yaw[i];
			B(21,i) = B_buffer.cmd[nextCmd].vyaw[i];
			B(23,i) = B_buffer.cmd[nextCmd].ayaw[i];

			T(0,i) = B_buffer.cmd[nextCmd].wait_time[i];
			T(1,i) = B_buffer.cmd[nextCmd].duration[i];
		}
	
		for(int i = 1; i < points; i++) {
			for(int j = 0; j < 4; j++) {
				B(6*j,i) = B(6*j+1,i-1);
				B(6*j+2,i) = B(6*j+3,i-1);
				B(6*j+4,i) = B(6*j+5,i-1);      
			}
		} 
		
		solveXMatrix();

		point = 0;
		c_time = 0.0;
		isComplete = false;		
	}
	return true;
}

bool Trajectory_Accel::isBufferFull()
{
 /* Buffer checker
  * - Returns true if buffer is full
  */

	for(int i=0; i<BUF; i++) {
		if(!B_buffer.isValid[i]) {
			return false;
		}
	}

	return true;
}

int Trajectory_Accel::nextBuffer()
{
 /* Returns the position of the next buffered command
  * - If buffer is empty, then nextBuffer() returns -1
  */

	int valid = -1;
	for(int i=B_buffer.buf_now; i<BUF; i++) {
		if(B_buffer.isValid[i]) {
			valid = i;
			return valid;
		}
	}

	for(int i=0; i<B_buffer.buf_now; i++) {
		if(B_buffer.isValid[i]) {
			valid = i;
			return valid;
		}
	}

	return valid;
}

int Trajectory_Accel::nextEmptyBuffer()
{
 /* Returns the position of the next empty buffer space
  * - If buffer is full, returns -1
  */

	int valid = -1;
	for(int i=B_buffer.buf_now; i<BUF; i++) {
		if(!B_buffer.isValid[i]) {
			valid = i;
			return valid;
		}
	}

	for(int i=0; i<B_buffer.buf_now; i++) {
		if(!B_buffer.isValid[i]) {
			valid = i;
			return valid;
		}
	}

	return valid;
}
