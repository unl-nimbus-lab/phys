#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "ohm_igvc/location_point.h"
#include "ohm_igvc/pid_feedback.h"
#include "ohm_igvc/planned_path.h"
#include "ohm_igvc/target.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <string>
#include <time.h>
#include <vector>
using namespace std;

typedef struct{
	double targdist;
	// double targbearing;
	// double front;
	// double right;
	double speed;               // between -1 to 1
	double turn;                // between -1 to 1
	double pErr;
	double lastpErr;
	double kP;
	double dErr;
	double kD;
	double iErr;
	double kI;
	// double lookAhead;
}CVAR;

ros::Publisher turnPub;
ros::Publisher feedbackPub;
CVAR cvar;
double lastTime, thisTime;
double maxIntErr = 0.5;
// double destinationThresh = 0.5;
ohm_igvc::target oldCurrentTarget;
double targetThreshold;
double targetChangeDelayParameter; //Hz
ohm_igvc::planned_path plannedPath;
geometry_msgs::Pose2D odometry;


double mathSign(double number){
	//Returns the number's sign
	//Equivalent to .NET's Math.Sign()
	//number>0 = 1
	//number=0 = 0
	//number<0 = -1
	if (number == 0){
		return 0;
	}
	else {
		return (number>0 ? 1 : -1);
	}
}

double adjust_angle(double angle, double circle){
	//circle = 2pi for radians, 360 for degrees
	// Subtract multiples of circle
	angle -= floor(angle / circle) * circle;
	angle -= floor(2 * angle / circle) * circle;

	return angle;
}

void initPID(){
	lastTime = ((double)clock()) / CLOCKS_PER_SEC;
	cvar.pErr = cvar.iErr = cvar.dErr = 0;
}

void pathPlanningCallback(const ohm_igvc::planned_path::ConstPtr& plannedPathMsg){	
	/* This fires every time a new planned path is published */
	plannedPath = *plannedPathMsg;
}

void odometryCallback(const geometry_msgs::Pose2D::ConstPtr& odometryMsg){	
	/* This fires every time a new position and heading is published */
	odometry = *odometryMsg;
}

void pathPlannerSpeedCallback(const geometry_msgs::Twist::ConstPtr& twistMsg){	
	/* This fires every time a new position and heading is published */
	cvar.speed = twistMsg->linear.x;
}

void pid(){
	if (oldCurrentTarget.latitude != plannedPath.currentTarget.latitude
		|| oldCurrentTarget.longitude != plannedPath.currentTarget.longitude){
		ros::Duration targetChangeDelay(targetChangeDelayParameter);
		targetChangeDelay.sleep();
		initPID();
	}
	oldCurrentTarget = plannedPath.currentTarget;

	double heading = odometry.theta;
	int dir = plannedPath.dir;
	double dx, dy, s, c, nx, ny, dt, temp;
	double dlastx, dlasty, lastDist;
	double desiredAngle;

	// int reachedTarget;

	if (dir < 0){
		//If we're going backwards we need to flip heading for calculations. Not used in IGVC
		heading = heading - M_PI * mathSign(heading);
	}

	dx = plannedPath.currentTarget.latitude - odometry.x;
	dy = plannedPath.currentTarget.longitude - odometry.y;

	dlastx = plannedPath.lastTarget.latitude - odometry.x;
	dlasty = plannedPath.lastTarget.longitude - odometry.y;
	lastDist = sqrt(dlastx*dlastx + dlasty*dlasty);
	
	// c = cos(heading);
	// s = sin(heading);

	cvar.targdist = sqrt(dx*dx + dy*dy);
	// cvar.right = dx*c - dy*s;
	// cvar.front = dy*c + dx*s;
	desiredAngle = adjust_angle(atan2(dx,dy), 2.0 * M_PI);

	nx = -(plannedPath.currentTarget.longitude-plannedPath.lastTarget.longitude); // -Delta Y
	ny =  (plannedPath.currentTarget.latitude-plannedPath.lastTarget.latitude); //  Delta X
	temp = hypot(nx, ny)+1e-3; // To avoid division by zero
	nx /= temp;
	ny /= temp;

	thisTime = ((double)clock()) / CLOCKS_PER_SEC;
	dt = thisTime - lastTime;

	/* Current Target Heading PID Calculations
	 * Calculate between current heading and heading to target
	 * Use kP, kD, kI weights to get turn variable used to set
	 * left/right wheel speeds
	 */
	cvar.lastpErr = cvar.pErr;
	cvar.pErr = adjust_angle(heading - desiredAngle, 2.0 * M_PI); //Calculate Proportional Error
	cvar.iErr = cvar.iErr + cvar.pErr * dt; //Calculate Integral Error
	cvar.iErr = mathSign(cvar.iErr) * fmin(abs(cvar.iErr), maxIntErr); //Limit max integral error since this can go to infinity
	if (dt != 0) cvar.dErr = (cvar.pErr - cvar.lastpErr) / dt; //Calculate Derivative Error. Check that some time has passed so a divide by 0 error doesn't happen
	cvar.turn = -(cvar.kP * cvar.pErr + cvar.kI * cvar.iErr + cvar.kD * cvar.dErr); //Calculate turn form errors and weights

	lastTime = thisTime; //set time variable

	// inLastTarget = (lastDist<leaveTargetThresh);//Still in last zone.
	// approachingTarget = (cvar.targdist < approachingThresh);//Getting close to the point
	// reachedTarget = (cvar.targdist < destinationThresh);//Target threshold has been reached
	ohm_igvc::pid_feedback feedbackMsg;
	feedbackMsg.lastDist = lastDist;
	feedbackMsg.targetDist = cvar.targdist;
	feedbackPub.publish(feedbackMsg);

	geometry_msgs::Twist msg;
	msg.linear.x = cvar.speed;
	msg.angular.z = cvar.turn;
	turnPub.publish(msg);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "ohm_pid");

	ros::NodeHandle n;

	n.param("pid_target_threshold", targetThreshold, 5.0);
	n.param("pid_target_change_delay", targetChangeDelayParameter, 0.0);

	feedbackPub = n.advertise<ohm_igvc::pid_feedback>("ohmPidFeedback", 5);
	turnPub = n.advertise<geometry_msgs::Twist>("ohmPid", 5);

	initPID();

	ros::Subscriber pathPlanningSub = n.subscribe("pathPlanning", 5, pathPlanningCallback);
	ros::Subscriber odometrySub = n.subscribe("/ohm/odom", 5, odometryCallback);
	ros::Subscriber pathPlannerSpeedSub = n.subscribe("/ohm/pathPlannerSpeed", 5, pathPlannerSpeedCallback);

	// ros::spin();
	while(ros::ok()){
		ros::spinOnce();
		pid();
	}
	
	return 0;
}