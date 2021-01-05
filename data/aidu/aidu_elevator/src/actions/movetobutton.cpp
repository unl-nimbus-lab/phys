#include <ros/ros.h>
#include <aidu_elevator/actions/movetobutton.h>
#include <geometry_msgs/Twist.h>
#include <aidu_robotarm/robot_arm_positions.h>
#include <aidu_vision/DistanceSensors.h>
#include <geometry_msgs/Twist.h>

#define HORIZONTAL_FOV 1.07115918
#define VERTICAL_FOV 0.644356361
#define SIGN(X) (X > 0 ? 1 : (X < 0 ? -1 : 0))
#define BOUND(MIN,X,MAX) (std::min(MAX,std::max(MIN,X)))

using namespace aidu::elevator;

double convert(double fov, double img_x, double z, double resolution) {
    double theta = -fov/2.0+(fov*img_x/resolution);
    double x = tan(theta)*z;
    return(x);
}


MoveToButton::MoveToButton(ros::NodeHandle* nh, int button) : Action::Action(nh) {
    //publisher
    positionPublisher = nh->advertise<geometry_msgs::Twist>("/pos",1,true);
    speedPublisher = nh->advertise<geometry_msgs::Twist>("/cmd_vel",1);
    robotArmPublisher = nh->advertise<aidu_robotarm::robot_arm_positions>("/robot_arm_positions",1,true);
    //subscriber
    robotArmSubscriber = nh->subscribe<sensor_msgs::JointState>("/arm_state", 1, &MoveToButton::updateArmState, this);
    sensorSubscriber = nh->subscribe("/sensors", 1, &MoveToButton::sensorCallback, this);
    buttonSubscriber = nh->subscribe("/elevator/button/classified", 1, &MoveToButton::buttonCallback, this);
    distance = -1.0;
    buttonX = -1000.0;
    this->button = button;
}

MoveToButton::~MoveToButton() {
    
}

void MoveToButton::execute() {
  aidu_robotarm::robot_arm_positions arm_positions;
  bool door_open=false;
  ROS_INFO("Executing action: move to button");
  ros::Rate loop(20);
  
  // Wait for distance
  ROS_INFO("Waiting for distance measurement");
  while(ros::ok() && distance == -1.0) {
      ros::spinOnce();
      loop.sleep();
  }
  
  // Wait for button
  ROS_INFO("Waiting for button");
  while(ros::ok() && buttonX == -1000.0) {
      ros::spinOnce();
      loop.sleep();
  }
  
  // Move
  ROS_INFO("Moving base");
  sleep(1);
  
  // Translate arm to button y position
  ROS_INFO("buttonY: %f ",buttonY);
  arm_positions.translation=translation-buttonY;
  ROS_INFO("Arm translation: %f", arm_positions.translation);
  robotArmPublisher.publish(arm_positions);
  ros::spinOnce();
  sleep(3);
  
  // Correct base position
  if(fabs(buttonX) > 0.04) {
    this->moveBase(0.0, SIGN(buttonX) * 1.57);
    sleep(3);
    this->moveBase(fabs(buttonX), 0.0);
    sleep(4);
    this->moveBase(0.0,  -SIGN(buttonX) * 1.57);
    sleep(3);
  }
  
  // Move forward with P control
  double target = 0.07;
  geometry_msgs::Twist twist;
  while(ros::ok() && fabs(distance - target) > 0.01) {
    double error = distance - target;
    double Kp = 1.0;
    ROS_INFO("error: %f, bounded error*Kp: %f, error*Kp: %f", error, BOUND(-0.05,error*Kp,0.05), error*Kp);
    twist.linear.x = BOUND(-0.05, error*Kp, 0.05);
    speedPublisher.publish(twist);
    ros::spinOnce();
    loop.sleep();
  }
  twist.linear.x = 0.0;
  speedPublisher.publish(twist);
  ros::spinOnce();
  
  // Done!
  sleep(5);
  this->inFrontOfButton = true;
  ROS_INFO("In front of button");
}

void MoveToButton::moveBase(double linear, double angular) {
    geometry_msgs::Twist position;
    position.angular.z = angular;
    position.linear.x = linear;
    positionPublisher.publish(position);
    ros::spinOnce();
}

bool MoveToButton::finished() {
  return inFrontOfButton;
}

void MoveToButton::sensorCallback(const aidu_vision::DistanceSensors::ConstPtr& dist_msg) {
    distance = dist_msg->arm / 1000.0;
    //ROS_INFO("distance:%f",distance);
}

void MoveToButton::buttonCallback(const aidu_elevator::Button::ConstPtr& message) {
    if (message->button_type == button) {
        buttonX = convert(HORIZONTAL_FOV, message->x, distance, 1280);
	buttonY = convert(VERTICAL_FOV, message->y, distance, 720);
	ROS_INFO("buttonX %f  pixelsX: %d", buttonX, message->x);
    }
}


void MoveToButton::updateArmState(const sensor_msgs::JointState::ConstPtr& message) {
    for(unsigned int i=0; i<message->name.size(); i++) {
        if (message->name[i] == "base_spindlecaddy") {
            translation = message->position[i];
	}
    }
}

