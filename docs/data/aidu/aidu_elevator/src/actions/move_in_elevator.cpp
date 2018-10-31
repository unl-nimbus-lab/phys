#include <ros/ros.h>
#include <aidu_elevator/actions/move_in_elevator.h>
#include <sensor_msgs/LaserScan.h>
#include <aidu_robotarm/robot_arm_positions.h>
#include <math.h>

#define BOUND(MIN,X,MAX) (std::min(MAX,std::max(MIN,X)))

using namespace aidu::elevator;

MoveInElevator::MoveInElevator(ros::NodeHandle* nh) : Action::Action(nh) {
    
    // Set up subscribers
    laserscanSubsciber = nh->subscribe("/laserscan", 1, &MoveInElevator::laserscancallback, this);
    sensor_sub = nh->subscribe("/sensors", 1, &MoveInElevator::sensorcallback, this);
    // Set up publishers
    basepositionPublisher= nh->advertise<geometry_msgs::Twist>("/pos",1);
    speedPublisher = nh->advertise<geometry_msgs::Twist>("/cmd_vel",1);
    
    // Initialize variables
    rotationdirection=1.0;
    begining=true;
    action_finished=false;
    

    
}

MoveInElevator::~MoveInElevator() {
}

void MoveInElevator::execute() {
  ROS_INFO("executing Moving in elevator");
  
  // initializing variables 
  double target=0.31;
  double target2=0.55;
  double target_angle=0.0;;

  SetPos(1.5,0.0);
  begining=false;
  sleep(9);
  // position control from wall
  controlloop(target2,target_angle);
  
  
  if (rotationdirection==1){
    //move in elevator
    //ROS_INFO("closest wall is left");
    SetPos(0.0,-1.5707);
    sleep(3);
    
    // position control from wall
    controlloop(target,target_angle);
    
    //turn back
    SetPos(0.0,-1.572);
    sleep(3);
    
    //move forward
    SetPos(0.44,0.0);
    sleep(3);
  }
  else if (rotationdirection==-1){
    
  }
  action_finished=true;
      
}

bool MoveInElevator::finished() {
 return action_finished;
}

void MoveInElevator::controlloop(double target_dist, double target_angle){
  ros::Rate loop(20);
  geometry_msgs::Twist speed;
  double angle_error=10;
  double Kp=1.0;
  double angle_Kp=1.0;
  while(ros::ok() && (fabs(distance - target_dist) > 0.007 || angle_error>0.005 )) {
    double error = distance - target_dist;
    angle_error=front_left-front_right;
    //ROS_INFO("error: %f, bounded error*Kp: %f, error*Kp: %f", error, BOUND(-0.05,error*Kp,0.05), error*Kp);
    speed.linear.x = BOUND(-0.05, error*Kp, 0.05);
    speed.angular.z=BOUND(-0.1,angle_error*angle_Kp, 0.1);
    speedPublisher.publish(speed);
    ros::spinOnce();
    //ROS_INFO("control loop: speed:%f",speed.linear.x);
    loop.sleep();
    
  }
  speed.linear.x = 0.0;
  speedPublisher.publish(speed);
  ros::spinOnce();
  sleep(1);
  
}

void MoveInElevator::SetPos(double x, double angle){
    geometry_msgs::Twist position;
    position.linear.x=x;
    position.angular.z=angle;
    basepositionPublisher.publish(position);
    ros::spinOnce();
}

void MoveInElevator::sensorcallback(const aidu_vision::DistanceSensors::ConstPtr& dist_msg){
    front_left=dist_msg->Frontleft/1000.0; 
    front_right=dist_msg->Frontright/1000.0;
    dist_arm=dist_msg->arm/1000.0;
    distance=(front_left+front_right)/2.0;
}


void MoveInElevator::laserscancallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg){
  if (begining=true){
    int size =(scan_msg->angle_max-scan_msg->angle_min)/(scan_msg->angle_increment);
    double dist_treshold=1.5; //distance in meters
    float min_x=INFINITY;
    float max_x=-INFINITY;
    float* x = new float[size];
    float* y = new float[size];
    double angle =scan_msg->angle_min;
    for (int i=0;i<size;i++){
      x[i]=sin(angle)*scan_msg->ranges[i];
      y[i]=cos(angle)*scan_msg->ranges[i];
      angle+=scan_msg->angle_increment;
      if (y[i] > dist_treshold){
	min_x=std::min(min_x,x[i]);
	max_x=std::max(max_x,x[i]);
      }    
    }
    if (fabs(min_x) > max_x){
      rotationdirection=1;
    } else {
      rotationdirection=-1;
    }
    
    delete x;
    delete y;
    
  }
}






