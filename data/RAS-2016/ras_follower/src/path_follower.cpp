#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ras_follower/moveTo.h>
#include <ras_follower/Rotate.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <boost/math/special_functions/round.hpp>

#define ZERO 1E-1
/*
 * States: 0 idle
 *         1 Following
 *         -1 Error following
 *         2 Externally aborted
 *         8 rotating
 */
class PathFollower
{
public:
  ros::NodeHandle nH;
  ros::Subscriber point_subscriber;
  ros::Subscriber grid_subscriber;
  ros::Publisher twist_publisher;
  ros::Publisher marker_publisher;
  ros::Publisher state_publisher;
  ros::ServiceServer service;
  ros::ServiceServer abort_follow_service;
  ros::ServiceServer rotate_Service;
  ros::ServiceServer relative_rotate_Service;
  std_msgs::Int32 mStateMsg;
  nav_msgs::OccupancyGrid gridMap;
  tf::Vector3 mTarget;
  geometry_msgs::Pose mTargetPose;

  float windowSize;
  double a;
  double b;
  int l;
  float alpha; //rad
  int T;
  int s_max;
  int targetSector;
  int k_n;
  int k_f;
  float V_max;
  float h_m;
  float P_angle;
  float robotWidth;
  float mLengthFromCenterToFront;
  bool mErrorFollowing;
  int mMaxSearchWidth;
  double mTolerance;
  bool mShouldKillMotorsOnTarget;
  double mTargetAngle;

  int mState;
  const static int RUNNING = 1;
  const static int ROTATING_TO_POSE = 2;
  const static int DONE_ROTATING = 3;
  const static int ABORTED = 4;
  const static int IDLE = 5;
  const static int DONE = 6;

  std::vector<double> smothHist;

  tf::Stamped<tf::Pose> robotPoseInWorld;
  tf::Vector3 robotPointInGrid;
  tf::Transformer transFormer;

  PathFollower() :
                  nH("~"),
                  targetSector(18),
                  k_n(18),
                  k_f(18),
                  mErrorFollowing(false)
    {
    windowSize = 2;
    a = 2;
    b = 2;
    alpha =2*M_PI/72;
    l = 5;
    T = 100;
    s_max = 15;
    h_m = T*0.9;
    V_max = 0.2;
    P_angle = 1;
    robotWidth = 0.4;
    mLengthFromCenterToFront = 0.18;

    smothHist.resize((int) 2*M_PI/alpha,0);

    mStateMsg.data = 0;
    mState = IDLE;

    nH.getParam("window_size", windowSize);
    nH.getParam("a", a);
    nH.getParam("b", b);
    nH.getParam("alpha", alpha);
    nH.getParam("l", l);
    nH.getParam("T", T);
    nH.getParam("s_max", s_max);
    nH.getParam("V_max", V_max);
    nH.getParam("h_m", h_m);
    nH.getParam("P_angle", P_angle);
    nH.getParam("robot_width", robotWidth);
    nH.getParam("distance_between_front_and_center", mLengthFromCenterToFront);
    nH.getParam("max_search_width", mMaxSearchWidth);

    service = nH.advertiseService("/robot/goTo", &PathFollower::servicCallBack, this);
    abort_follow_service = nH.advertiseService("/robot/abort_follow_service", &PathFollower::abortServicCallBack, this);
    rotate_Service = nH.advertiseService("/robot/rotate_to_target", &PathFollower::rotateToTarget, this);
    relative_rotate_Service = nH.advertiseService("/robot/relative_rotate_to_target", &PathFollower::relRotateToTarget, this);
    point_subscriber = nH.subscribe("/localization/pose",1, &PathFollower::PoseCallback, this);
    grid_subscriber = nH.subscribe("/mapping/grid",1, &PathFollower::gridCallback, this);
    twist_publisher = nH.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    marker_publisher = nH.advertise<visualization_msgs::MarkerArray>("markers", 1);
    state_publisher = nH.advertise<std_msgs::Int32>("state",1);

    tf::Vector3 orig(0,0,0);
    this->robotPoseInWorld.setOrigin(orig);
  }
  
  bool abortServicCallBack(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    shutDownMotors();
    mState = IDLE;
    return true;
  }
  
  bool relRotateToTarget(ras_follower::Rotate::Request &req, ras_follower::Rotate::Response &rsp){
    tf::Quaternion angleQ = tf::createQuaternionFromYaw(req.target.angular.z + tf::getYaw(robotPoseInWorld.getRotation()));
    geometry_msgs::Pose p;
    tf::quaternionTFToMsg(angleQ, p.orientation);
    mTargetPose = p;
    mState = ROTATING_TO_POSE;
    mErrorFollowing = false;
    return true;

  }

  bool rotateToTarget(ras_follower::Rotate::Request &req, ras_follower::Rotate::Response &rsp){
      double targetAngleInWorld = std::atan2(req.target.linear.y - robotPoseInWorld.getOrigin().getY(),
                                             req.target.linear.x - robotPoseInWorld.getOrigin().getX());
      tf::Quaternion angleQ = tf::createQuaternionFromYaw(targetAngleInWorld);
      geometry_msgs::Pose p;
      tf::quaternionTFToMsg(angleQ, p.orientation);
      mTargetPose = p;
      mState = ROTATING_TO_POSE;
      mErrorFollowing = false;
      return true;
  }

  void gridCallback(const nav_msgs::OccupancyGridConstPtr &grid){
    //Assume grid is in world.
    gridMap = *grid;
  }

  void PoseCallback(const geometry_msgs::PoseStampedConstPtr &pose) {
      tf::poseStampedMsgToTF(*pose, this->robotPoseInWorld);
  }

  bool servicCallBack(ras_follower::moveTo::Request &req,
                      ras_follower::moveTo::Response &res) {
    mErrorFollowing = false;
    mTarget = tf::Vector3(req.goal.pose.position.x, req.goal.pose.position.y, 0);
    mTargetPose = req.goal.pose;
    mTolerance = req.tolerance;
    mShouldKillMotorsOnTarget = req.turnOffMotors.data;
    mState = RUNNING;
    return true;
  }

  void mainLoop(){
    ros::Rate tRate(10);
    mState = IDLE;
    while(nH.ok()){
      float velocity;
      float angle;
      tf::Quaternion tfQ;
      tRate.sleep();
      ros::spinOnce();
      switch (mState) {
      case IDLE:
        mStateMsg.data = 0;
        state_publisher.publish(mStateMsg);
        break;
      case RUNNING:
        mStateMsg.data = 1;
        state_publisher.publish(mStateMsg);
        ROS_INFO("Calculating Vel and angle...");
        robotPointInGrid = tf::Vector3(gridMap.info.resolution*gridMap.info.width/2,gridMap.info.resolution*gridMap.info.height/2, 0 );
        this->calculateVelocity(mTarget, velocity, angle);
        broadcastSections();
        ROS_INFO("Tol: %f", mTolerance);
        if(isRobotOnTarget(mTarget, mTolerance)) {
          //Sucess!
          if(mShouldKillMotorsOnTarget){
            mState = ROTATING_TO_POSE;
            continue;
          } else {
            mState = IDLE;
            continue;
          }
        } else if (mErrorFollowing) {
          //Fail...
          shutDownMotors();
          mStateMsg.data =  -1;
          state_publisher.publish(mStateMsg);
          mState = IDLE;
          continue;
        }
        ROS_INFO("Sending Vel: %f, and angle: %f", velocity, angle);
        this->sendForceToMotorController(velocity,angle);
        break;
      case ROTATING_TO_POSE:
        mStateMsg.data = 8;
        state_publisher.publish(mStateMsg);
        tf::quaternionMsgToTF(mTargetPose.orientation, tfQ);
        mTargetAngle = tf::getYaw(tfQ);
        angle =mTargetAngle- tf::getYaw(robotPoseInWorld.getRotation());
        if(std::abs(angle) < M_PI/50){
          shutDownMotors();
          mState = IDLE;
          continue;
        }
        velocity = 0;
        ROS_INFO_STREAM("target: " << mTargetAngle << ", angle: " << tf::getYaw(robotPoseInWorld.getRotation()) << ", diff: " << angle);
        ROS_INFO("Sending Vel: %f, and angle: %f", velocity, angle);
        this->sendForceToMotorController(velocity,angle);
        break;
      case ABORTED:
        mStateMsg.data = 2;
        state_publisher.publish(mStateMsg);
        mState = IDLE;
        break;
      default:
        break;
      }
    }
    shutDownMotors();
  }

  bool isRobotOnTarget(tf::Vector3& targetVector, float tolerance){
    bool answer = (this->robotPoseInWorld.getOrigin() - targetVector).length() < tolerance;
    tf::Transform robotCenterFromWorldTransform;
    robotCenterFromWorldTransform.setOrigin(robotPoseInWorld.getOrigin());
    robotCenterFromWorldTransform.setRotation(robotPoseInWorld.getRotation());
    tf::Vector3 tipPose = robotCenterFromWorldTransform(tf::Vector3(mLengthFromCenterToFront,0,0));
    answer |=   (tipPose - targetVector).length() < tolerance;
    return answer;
  }

  void broadcastSections() {
    int maxK = (int) 2*M_PI/alpha;
    visualization_msgs::MarkerArray allSections;
    allSections.markers.resize(maxK);
    for (int k = 0; k < maxK; ++k) {
      visualization_msgs::Marker aMarker;
      tf::Stamped<tf::Pose> markerPose;
      markerPose.setOrigin(robotPointInGrid);
      markerPose.setRotation(tf::createQuaternionFromYaw(k*alpha));
      geometry_msgs::PoseStamped aPose;
      tf::poseStampedTFToMsg(markerPose, aPose);
      aMarker.pose = aPose.pose;
      aMarker.type = aMarker.ARROW;
      aMarker.color.a = 0.6;
      aMarker.scale.x = 0.5;
      aMarker.scale.y = 0.01;
      aMarker.scale.z = 0.01;
      aMarker.header.frame_id = "/live_map";
      aMarker.header.stamp = ros::Time::now();
      aMarker.id = k;
      aMarker.ns = "sectors";
      if(smothHist[k] > T){
        aMarker.color.r = 1;
      } else {
        aMarker.color.g = 1;
      }
      allSections.markers[k] = aMarker;
    }
    marker_publisher.publish(allSections);
  }

  void getSmoothPolarHistogram(std::vector<double>& smothHist, tf::Vector3& targetVector)
  {
    int cellMinX = int((robotPointInGrid.getX() - windowSize/2)/gridMap.info.resolution);
    int cellMinY = int((robotPointInGrid.getY() - windowSize/2)/gridMap.info.resolution);
    int cellMaxX = int((robotPointInGrid.getX() + windowSize/2)/gridMap.info.resolution);
    int cellMaxY = int((robotPointInGrid.getY() + windowSize/2)/gridMap.info.resolution);
    if(cellMinX<0){
      cellMinX = 0;
    }
    if(cellMinY<0){
      cellMinY = 0;
    }
    if(gridMap.info.width <= cellMaxX){
      cellMaxX = gridMap.info.width-1;
    }
    if(gridMap.info.height <= cellMaxY){
      cellMaxY = gridMap.info.height-1;
    }
    for (int Ycell = cellMinY ; Ycell <= cellMaxY; ++Ycell) {
      for (int Xcell = cellMinX; Xcell <= cellMaxX; ++Xcell) {
        int rowNr = Ycell*gridMap.info.width + Xcell;
        float x = Xcell*gridMap.info.resolution;
        float y = Ycell*gridMap.info.resolution;
        tf::Vector3 objectVec(x,y,0);
        objectVec = objectVec - robotPointInGrid; //Moves the vector to be robot -> object.
        //See report @ <url:http://www-personal.umich.edu/~johannb/Papers/paper16.pdf/>
        float beta = std::atan2((y - robotPointInGrid.getY()),(x - robotPointInGrid.getX()));
        float distance = sqrt(std::pow(y - robotPointInGrid.getY(),2) + std::pow(x - robotPointInGrid.getX(), 2));
        float extraAngle = std::atan2(robotWidth/2, distance);
        float m = std::pow(gridMap.data[rowNr], 2)*(a-b*distance);
        for (int sector = (int)( (beta-extraAngle)/alpha); sector <= (int)((beta+extraAngle)/alpha); sector++) {
          if(objectVec.dot(targetVector)/targetVector.length() < targetVector.length())
            smothHist[mod(sector,smothHist.size())] += m;
        }
      }
    }
  

  }

  void getKnKfWithTargetInside(int targetSector, std::vector<double> &smothHist, int &k_f, int &k_n)
  {
    k_n = targetSector;
    k_f = targetSector;
  }

  void getKnKfWithTargetOutside(std::vector<double> &smothHist, int targetSector,int& k_n, int& k_f)
  {
    int i = 1;
    bool haveNotFoundGoodVallyStart = true;
    bool goingRight = true;

    while(haveNotFoundGoodVallyStart) {
      int upIndex = targetSector + i;
      int downIndex = targetSector - i;
      if(smothHist[mod(upIndex,smothHist.size())] < T){
        haveNotFoundGoodVallyStart = false;
        k_n = upIndex;
        goingRight = false;
      } else if(smothHist[mod(downIndex,smothHist.size())] < T){
        haveNotFoundGoodVallyStart = false;
        k_n = downIndex;
        goingRight = true;
      }
      if(i > mMaxSearchWidth){ //We have searched the hole space and can not move... :(
        mErrorFollowing = true;
        return;
      }
      i++;
    }
    int direction;
    if(goingRight){
      direction = -1;
    } else {
      direction = 1;
    }

    bool haveNotFoundEnd = true;
    i = 0;
    while(haveNotFoundEnd) {
      int index = k_n + i;
      if(smothHist[mod(index,smothHist.size())] > T){
        haveNotFoundEnd = false;
        k_f = index;
      }
      i += direction;
    }

    if(abs(k_f-k_n) > s_max){
      if(goingRight){
        k_f = k_n - s_max;
      } else {
        k_f = k_n + s_max;
      }
    }
    return;
  }

  void calculateVelocity(tf::Vector3 &target, float &velocity, float &angle) {
    //Look for okey vally:
    tf::Transform transform;
    transform.setOrigin(robotPoseInWorld.getOrigin());
    transform.setRotation(robotPoseInWorld.getRotation());
    transform = transform.inverse();
    tf::Vector3 targetInRobotCenter = transform(target);
    targetSector = int(atan2(targetInRobotCenter.getY(), targetInRobotCenter.getX())/alpha);
    if(std::isnan(targetSector)){
      return;
    }
    //Create circle histogram
    std::fill(smothHist.begin(), smothHist.end(), 0);
    getSmoothPolarHistogram(smothHist, targetInRobotCenter);

    bool targetIsInSection = smothHist[mod(targetSector,smothHist.size())] < T;
    //Check if target is in good vally, if so look for edges.
    if(targetIsInSection) {
      getKnKfWithTargetInside(targetSector, smothHist, k_f, k_n);
    } else {
      getKnKfWithTargetOutside(smothHist, targetSector,k_n, k_f);
      if(mErrorFollowing)
        return;
    }
    //We have sektin k_n -> k_f
    //Time to calculate direction.
    int theata_sector = int((k_n + k_f)/2);

     //Calculate speed
    float h_prim = smothHist[mod(theata_sector,smothHist.size())];
    float h_primprim = std::min(h_prim, h_m);
    float V_prim = V_max*(1-h_primprim/h_m);
    //Return force
    velocity = V_prim;
    angle = theata_sector*alpha;
  }

  void sendForceToMotorController(float velocity, float angle) {
    geometry_msgs::Twist controllTwist;
    double angleDiff = mod2(angle, 2*M_PI);
    if(abs(angleDiff)> M_PI){
      if(angleDiff< 0){
        angleDiff += 2*M_PI;
      } else {
        angleDiff -= 2*M_PI;
      }
    }
    controllTwist.linear.x = velocity*std::cos(angleDiff);
    controllTwist.angular.z =P_angle*(angleDiff);
    controllTwist.angular.z = std::min(std::max(controllTwist.angular.z,-2.0),2.0);

    if(smothHist[0] >  T || controllTwist.linear.x < 0) {
		controllTwist.linear.x = 0;
    	if(controllTwist.angular.z > 0)
            controllTwist.angular.z = std::max(controllTwist.angular.z, 0.5);
    	else if(controllTwist.angular.z < 0)
            controllTwist.angular.z = std::min(controllTwist.angular.z, -0.5);
        mStateMsg.data = 8;
        state_publisher.publish(mStateMsg);
    } else {
    	mStateMsg.data = 1;
        state_publisher.publish(mStateMsg);
    }
    twist_publisher.publish(controllTwist);
  }
  void shutDownMotors() {
    geometry_msgs::Twist controllTwist;
    controllTwist.linear.x = 0;
    controllTwist.angular.z = 0;
    twist_publisher.publish(controllTwist);
  }

  int mod(int a, int b)
  { return (a%b+b)%b; }

  float mod2(float a, float b)
  { return fmod((fmod(a,b)+b),b); }

};

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "path_follower");
  PathFollower pF;
  pF.mainLoop();
}
