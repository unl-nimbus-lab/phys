#ifndef ROMEOGRASPINGOBJECT_H
#define ROMEOGRASPINGOBJECT_H

// ROS Headers
#include <ros/ros.h>
/*
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>
#include <alcommon/alproxy.h>
#include <alcommon/albrokermanager.h>
#include <alproxies/almotionproxy.h>
*/
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <object_tracker_msg_definitions/ObjectInfo.h>
//#include <std_msgs/Float32.h>
//#include <moveit_msgs/DisplayTrajectory.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "romeo_moveit_actions/metablock.hpp"
#include "romeo_moveit_actions/action.hpp"
//#include <metablock.hpp>
//#include <action.hpp>
#include "romeo_grasper/modeledobject.h"
#include "romeo_grasper/VisualTable.h"
#include "romeo_grasper/romeosimulator.h"

#include <moveit/move_group_interface/move_group.h>
//#include <moveit/planning_scene_interface/planning_scene_interface.h>

// General parameters
#define PARAM_VERBOSE                           1
#define PARAM_MOVE_GROUP                        2
#define PARAM_POSE_THRESHOLD                    3
#define PARAM_CONFIDENCE_THRESHOLD              4
#define PARAM_REFERENCE_CONFIDENCE_THRESHOLD    5
#define PARAM_SIMULATION                        6
#define PARAM_AUTOMATIC_EXECUTION               7
#define PARAM_REACH_VS_GRASP                    8
// Parameters for object recognition
#define PARAM_MODEL_OBJECT_NAME                 10
#define PARAM_MODELS_DIRECTORY                  11
#define PARAM_TRACKING                          12
#define PARAM_CAMERA_FRAME_ID                   13
#define PARAM_CAMERA_REFERENE_FRAME_ID          14
#define PARAM_MODEL_REFERECE_NAME               15
#define PARAM_POSE_CAMERA_PREKNOWN              16
#define PARAM_OBJECT_FRAME_ID                   17
#define PARAM_BASE_LINK                         18
#define PARAM_CAMERA_IN_FRONT                   19
#define PARAM_CAMERA_LINK                       20
// Parameters of Action class
#define PARAM_MAX_VELOCITY_SCALE_FACTOR         31
#define PARAM_PLANNING_TIME                     32
#define PARAM_TOLERANCE_STEP                    33
#define PARAM_TOLERANCE_MIN                     34
#define PARAM_ATTEMPTS_MAX                      35
#define PARAM_ACTION_FLAG                       36

#define ANSWER_SRV_WAIT     0
#define ANSWER_SRV_MOVE     1
#define ANSWER_SRV_REPLAN   2
#define ANSWER_SRV_ABORT    3

#define LOAD_PARAMS_ALL     0
#define LOAD_PARAMS_RUN     1

#define CAMERA_POSED_NEXT_TO            0
#define CAMERA_POSED_IN_FRONT           1

using std::string;

static const double TABLE_HEIGHT = 0.75;
static const double TABLE_WIDTH  = 1.2;
static const double TABLE_DEPTH  = 1.2;
static const double TABLE_X = 0.9;
static const double TABLE_Y = 0;

static const string SUPPORT_SURFACE3_NAME = "table";

class RomeoGrasper
{
private:

    //ROS Standard Variable
    ros::NodeHandle node_handle_;
	string ns_;

    // ROS Topics/Messages
    ros::ServiceClient change_model_service_client_;
    ros::Subscriber obj_pose_sub_;
    ros::Subscriber tracker_confidence_sub_;    
    ros::Subscriber visual_table_sub_;
    ros::Subscriber trajectory_sub_;
    ros::ServiceServer execute_service_;
    ros::ServiceServer replan_service_;
    ros::ServiceServer abort_service_;
    ros::Publisher plan_pub_;
    //ros::Publisher display_publisher_;
    //moveit_msgs::DisplayTrajectory display_trajectory_;
    geometry_msgs::PoseStamped pose_target_stamped_;
    geometry_msgs::PoseStamped camera_pose_;
    moveit_msgs::RobotTrajectory trajectory_;

    boost::shared_ptr<boost::thread> transform_thread_;

    // Robot Parameters
    string move_group_;
    string model_object_name_;
    string model_reference_name_;
    string models_directory_;
    string current_model_name_;
    string camera_reference_frame_id_;
    string camera_link_;
    string camera_frame_id_;
    string object_frame_id_;
    string base_link_;
    float confidence_threshold_;
    float pose_threshold_;
    double x_pose_camera_, y_pose_camera_, z_pose_camera_;
    //Member function setMaxAccelerationScalingFactor not finded (I dont know why)
    //float max_acceleration_scaling_factor_;
    float max_velocity_scaling_factor_, tolerance_step_, tolerance_min_, planning_time_;
    int attempts_max_;
    bool has_pose_;
    bool changed_pose_;
    bool is_busy_;
    bool enough_confidence_;
    bool camera_positioned_;
    bool preGraspVsPick;
    bool reachVsGrasp_;
    bool firstSetup_;
    bool waiting_service_;
    int answer_service_;

    bool verbose_;
    bool tracking_;
    bool camera_in_front_;
    bool pose_camera_preknown_;
    bool simulation_;
    bool automatic_execution_;

    bool is_visual_tools_setup_;
    bool is_actions_setup_;

    moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

    //MetaBlock *block_;
    ModeledObject *modeled_object_;
    moveit_simple_actions::Action *action_right_;
    moveit_simple_actions::Action *action_left_;

    boost::shared_ptr<RomeoSimulator> romeo_simulator_;
/*
    // Create your own broker
    boost::shared_ptr<AL::ALBroker> broker_;
    boost::shared_ptr<AL::ALMotionProxy> motionProxy_;
*/
    //moveit::planning_interface::MoveGroup moveItSetup();

    void objectTrackerSetup();
    void simpleGrasperSetup();
    void setupActions();
    void poseActionsInit();
    void setupVisualTools();
    void setActionParams(moveit_simple_actions::Action* current_action);

    void callbackTrajectory(moveit_msgs::RobotTrajectory data);
    void callbackObjectPose(object_tracker_msg_definitions::ObjectInfo data);
    void callbackVisualTable(romeo_grasper::VisualTable data);

    //void callbackTrackerConfidence(std_msgs::Float32 data);

    bool executePlan(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);
    bool rePlan(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);
    bool abortPlan(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);

    void loadParam(string param_name, float *param, float default_value, int param_flag);
    void loadParam(string param_name, int *param, int default_value, int param_flag);
    void loadParam(string param_name, string *param, string default_value, int param_flag);
    void loadParam(string param_name, bool *param, bool default_value, int param_flag);
    void changedParam(int param_flag);
    moveit_simple_actions::Action * currentAction();

    void publishTransforms();

    void findCameraReference();

    void changeTrackingModel(string model_filename);

    std::vector< geometry_msgs::Point> tableMesuramentsToPoints(double x, double y, double width, double height, double depth, double floor_to_base_height);
/*
    void headStiffness(float movement_time);

    bool naoqiSetup();
*/
public:
    RomeoGrasper();
    ~RomeoGrasper();

    void setup();

    void run();

    void planningAndExecutePoseGoal();

    void loadParams(int flag);

    void exit();
};

#endif // ROMEOGRASPINGOBJECT_H
