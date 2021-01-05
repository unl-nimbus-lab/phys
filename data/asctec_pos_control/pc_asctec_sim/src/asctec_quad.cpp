#include "asctec_quad.h"

AscTec_Quad::AscTec_Quad(string qframe, string wframe, tf::TransformListener *listener, float dt, struct K_DATA * kvals):
controller(qframe, wframe, kvals), accTraj(dt) 
{
	qframe_ = qframe;
	wframe_ = wframe;
	listener_ = listener;

	P_ptr = new PUB_DATA;
	W_ptr = new WAYPOINT;
	Q_ptr = new QUAD_OUT;
}

QUAD_OUT * AscTec_Quad::runQuad(QUAD_CMD * cmd)
{
	//Update params, start status, and battery value
	P_ptr->k_val = cmd->kvals;
	P_ptr->running = cmd->start;
	P_ptr->battery = cmd->battery;
	P_ptr->xyFree = cmd->xyFree;

	//Grab new transform data
	tf::StampedTransform transform;
	listener_->lookupTransform(wframe_, qframe_, ros::Time(0), transform);

	if(cmd->xyFree == 2) {
		P_ptr->vx = cmd->vx;
		P_ptr->vy = cmd->vy;
		transform.setOrigin(tf::Vector3(cmd->x, cmd->y, transform.getOrigin().getZ()));
	}

	//Run controller
	P_ptr = controller.runAsctec(P_ptr,&goal_,&transform);

	//If new trajectory is heard, set and update BMatrix
	if(cmd->newPath) {
		NEW_PATH path;
		path.cmd = cmd->f_path;
		path.state = P_ptr->state;
		path.type = buffer;
		accTraj.setBMatrix(&path);
	}

	//Follow trajectory flight path if all axes controlled
	Q_ptr->isComplete = accTraj.getComplete();
	if(!cmd->xyFree) {
		if(!accTraj.getComplete()) {
			W_ptr = accTraj.updateWaypoint(W_ptr);
			if(W_ptr->isValid) {
				goal_.goal = W_ptr->goal;
				goal_.isNew = true;
				Q_ptr->goal.x = W_ptr->goal.x;
				Q_ptr->goal.y = W_ptr->goal.y;
				Q_ptr->goal.z = W_ptr->goal.z;
			}
		}else {
			goal_.isNew = false;
		}
	}

	//Try manual controller setting
	Q_ptr->goalUpdated = setQuadGoal(&cmd->gNew);

	//Set return state
	Q_ptr->state = P_ptr->state;

	//Set TRPY command from controller
	Q_ptr->TRPYcmd = P_ptr->TRPYcmd;

	//Set TRPY controls to XY commands
	if(cmd->xyFree == 1) {
		Q_ptr->TRPYcmd.roll = controller.limitOutput(cmd->xyCmd.angular.x,XY_LIMIT,-XY_LIMIT);
		Q_ptr->TRPYcmd.pitch = controller.limitOutput(cmd->xyCmd.angular.y,XY_LIMIT,-XY_LIMIT);

	}
	return Q_ptr;
}

bool AscTec_Quad::setQuadGoal(struct GOAL_DATA * g)
{
	if(accTraj.getComplete() && (P_ptr->xyFree == 0 || P_ptr->xyFree == 2)) {
		if(g->isNew) {
			controller.updateGoal(g);
			Q_ptr->goal.x = g->goal.x;
			Q_ptr->goal.y = g->goal.y;
			Q_ptr->goal.z = g->goal.z;
			Q_ptr->goal.yaw = g->goal.yaw;
		}
		return true;
	}
	return false;
}
