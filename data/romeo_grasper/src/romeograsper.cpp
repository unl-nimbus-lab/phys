#include "romeo_grasper/romeograsper.h"

#include <ros/console.h>
#include <math.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include <object_tracker_srv_definitions/change_tracking_model.h>
#include <tf/transform_broadcaster.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

RomeoGrasper::RomeoGrasper()
{
    ros::NodeHandle nh("~");
    node_handle_ = nh;    
    ns_ = node_handle_.getNamespace();

    bool debug;
    node_handle_.param("debug", debug, false);
    if(debug)
    {
        if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
            ros::console::notifyLoggerLevelsChanged();
            // If debug is on the verbose info should too
            node_handle_.setParam("verbose", true);
        }
    }

    bool launch_full;
    node_handle_.param("launch_full", launch_full, false);

    if(launch_full)
    {
        /* This sleep is ONLY to allow Rviz to come up */
        ROS_INFO("RomeoGrasper waiting for Rviz...");
        sleep(10.0);
    }

    waiting_service_ = true;
    answer_service_ = ANSWER_SRV_WAIT;

    is_busy_ = false;
    has_pose_ = false;
    enough_confidence_ = false;
    changed_pose_ = false;
    camera_positioned_ = false;

    // True means have to do the pregrasp and false have to do the pick
    preGraspVsPick = true;

    transform_thread_ = boost::shared_ptr<boost::thread>(new boost::thread (boost::bind(&RomeoGrasper::publishTransforms, this)));

    firstSetup_ = true;
    is_visual_tools_setup_ = false;
    is_actions_setup_ = false;

    //Recommended use of the launchers and config to load correctly the parameters
    loadParams(LOAD_PARAMS_ALL);

    //naoqiSetup();

    setup();
    firstSetup_ = false;

    run();
}

RomeoGrasper::~RomeoGrasper()
{
    exit();
}

void RomeoGrasper::loadParams(int flag)
{
    if(flag == LOAD_PARAMS_RUN || flag == LOAD_PARAMS_ALL)
    {
        loadParam("move_group",
                  &move_group_,
                  "arm_hand_left",
                  PARAM_MOVE_GROUP);

        loadParam("pose_threshold",
                  &pose_threshold_,
                  0.05f,
                  PARAM_POSE_THRESHOLD);

        loadParam("confidence_threshold",
                  &confidence_threshold_,
                  0.25f,
                  PARAM_CONFIDENCE_THRESHOLD);

        loadParam("verbose",
                  &verbose_,
                  false,
                  PARAM_VERBOSE);

        loadParam("max_velocity_scaling_factor",
                  &max_velocity_scaling_factor_,
                  0.1f,
                  PARAM_MAX_VELOCITY_SCALE_FACTOR);

        loadParam("tolerance_step",
                  &tolerance_step_,
                  0.1f,
                  PARAM_TOLERANCE_STEP);

        loadParam("tolerance_min",
                  &tolerance_min_,
                  0.1f,
                  PARAM_TOLERANCE_MIN);

        loadParam("planning_time",
                  &planning_time_,
                  20.0f,
                  PARAM_PLANNING_TIME);

        loadParam("attempts_max",
                  &attempts_max_,
                  3,
                  PARAM_ATTEMPTS_MAX);

        loadParam("model_object_name",
                  &model_object_name_,
                  "dino",
                  PARAM_MODEL_OBJECT_NAME);

        loadParam("models_directory",
                  &models_directory_,
                  "/home/lluis/catkin_ws/src/romeo_grasper/data/models/",
                  PARAM_MODELS_DIRECTORY);

        loadParam("automatic_execution",
                  &automatic_execution_,
                  false,
                  PARAM_AUTOMATIC_EXECUTION);

        loadParam("reachVsGrasp",
                  &reachVsGrasp_,
                  false,
                  PARAM_REACH_VS_GRASP);
    }

    if(flag == LOAD_PARAMS_ALL)
    {
        loadParam("simulation",
                  &simulation_,
                  false,
                  PARAM_SIMULATION);

        loadParam("camera_in_front",
                  &camera_in_front_,
                  CAMERA_POSED_NEXT_TO,
                  PARAM_CAMERA_IN_FRONT);

        loadParam("tracking",
                  &tracking_,
                  false,
                  PARAM_TRACKING);

        loadParam("camera_frame_id",
                  &camera_frame_id_,
                  "/camera_rgb_optical_frame",
                  PARAM_CAMERA_FRAME_ID);

        loadParam("camera_link",
                  &camera_link_,
                  "/camera_link",
                  PARAM_CAMERA_LINK);

        loadParam("object_frame_id",
                  &object_frame_id_,
                  "/object_frame",
                  PARAM_OBJECT_FRAME_ID);

        loadParam("model_reference_name",
                  &model_reference_name_,
                  "/data/models/reference_name/tracking_model.ao",
                  PARAM_MODEL_REFERECE_NAME);

        loadParam("camera_ref_frame_id",
                  &camera_reference_frame_id_,
                  "/l_wrist",
                  PARAM_CAMERA_REFERENE_FRAME_ID);

        loadParam("pose_camera_preknown",
                  &pose_camera_preknown_,
                  false,
                  PARAM_POSE_CAMERA_PREKNOWN);

        // TODO: Look which frame I should use
        loadParam("base_link_frame_id",
                  &base_link_,
                  "/base_link",
                  PARAM_BASE_LINK);
    }
}

void RomeoGrasper::setup()
{
    ROS_INFO("Starting setup...");

    //Done in the Action class on topic /trajectory
    //plan_pub_ = node_handle_.advertise<moveit_msgs::RobotTrajectory>("trajectory", 1000);
    trajectory_sub_ = node_handle_.subscribe("/trajectory", 10, &RomeoGrasper::callbackTrajectory, this);

    execute_service_ = node_handle_.advertiseService("execute_plan", &RomeoGrasper::executePlan, this);
    replan_service_ = node_handle_.advertiseService("replan", &RomeoGrasper::rePlan, this);
    abort_service_ = node_handle_.advertiseService("abort_plan", &RomeoGrasper::abortPlan, this);
    // (Optional) Create a publisher for visualizing plans in Rviz.
    //display_publisher_ = node_handle_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

    romeo_simulator_.reset(new RomeoSimulator(node_handle_, "/trajectory"));

    simpleGrasperSetup();

    objectTrackerSetup();

    // May be visual_tools with Loading Romeo is not setup yet and we have to wait
    // findCameraReference function needs to transform and for this it needs the Loading Romeo be finished
    while(!is_visual_tools_setup_ || !is_actions_setup_)
    {
        ROS_DEBUG_ONCE("Waiting for the setup of visual_tools and actions");
        sleep(0.5);
    }

    //poseActionsInit();

    findCameraReference();

    while(!is_actions_setup_)
    {
        ROS_INFO_ONCE("Waiting to actions setup");
        sleep(0.5);
    }

    ROS_INFO("Setup finished");
    //First load of MoveGroup, load all the robot
    // With Action is not necessary
    //moveit::planning_interface::MoveGroup group(move_group_);
}

void RomeoGrasper::simpleGrasperSetup()
{
    ROS_DEBUG_STREAM("Setup Grasper");

    boost::shared_ptr<boost::thread> visual_tools_thread_ = boost::shared_ptr<boost::thread>(new boost::thread (boost::bind(&RomeoGrasper::setupVisualTools, this)));

    boost::shared_ptr<boost::thread> actions_thread_ = boost::shared_ptr<boost::thread>(new boost::thread (boost::bind(&RomeoGrasper::setupActions, this)));

    //TODO: Start in action.poseHandInit()?

    modeled_object_ = new ModeledObject(&node_handle_,
                                        visual_tools_,
                                        model_object_name_,
                                        "modeledObject",
                                        pose_target_stamped_,
                                        false,
                                        false,
                                        verbose_);

    /*
    ros::Time timestamp = ros::Time::now();
    geometry_msgs::Pose pick_pose = pose_target_stamped_.pose;
    uint shapeType = shape_msgs::SolidPrimitive::CYLINDER;
    double size = 0.05;
    double size_l = 0.15;
    block_ = new MetaBlock("block",
                         timestamp,
                         pick_pose,
                         shapeType,
                         size,
                         size_l);
*/
}

void RomeoGrasper::poseActionsInit()
{
    // Cartesian Paths
    // ^^^^^^^^^^^^^^^
    // You can plan a cartesian path directly by specifying a list of waypoints
    // for the end-effector to go through. Note that we are starting
    // from the new start state above.  The initial pose (start state) does not
    // need to be added to the waypoint list.
    std::vector<geometry_msgs::Pose> waypoints_left;
    std::vector<geometry_msgs::Pose> waypoints_right;

    geometry_msgs::Pose target_pose;

    // TODO: Look for which orientation use
    target_pose = action_left_->getPose();
    //target_pose.orientation.w = 1;

    // TODO: Check if waypoints are feasible
    //Go up
    target_pose.position.x = 0.1;
    target_pose.position.y = 0.27;
    target_pose.position.z = -0.1;
    waypoints_left.push_back(target_pose);
    target_pose.position.y *= -1;
    waypoints_right.push_back(target_pose);

    // Final pose - Extend the arms
    target_pose.position.x = 0.18;
    target_pose.position.y = 0.27;
    target_pose.position.z = 0.0;
    waypoints_left.push_back(target_pose);
    target_pose.position.y *= -1;
    waypoints_right.push_back(target_pose);

    //action_left_->poseHandInit();
    //action_right_->poseHandInit();
    //action_left_->poseHandInitWaypoints(waypoints_left);
    //action_right_->poseHandInitWaypoints(waypoints_right);
}

void RomeoGrasper::setupActions()
{
    ROS_DEBUG_STREAM("Creating Action left");
    action_left_ = new moveit_simple_actions::Action(&node_handle_, visual_tools_, "left", "romeo");
    ROS_DEBUG_STREAM("Creating Action right");
    action_right_ = new moveit_simple_actions::Action(&node_handle_, visual_tools_, "right", "romeo");
    setActionParams(action_left_);
    setActionParams(action_right_);

    is_actions_setup_ = true;
}

void RomeoGrasper::setupVisualTools()
{
    ROS_DEBUG_STREAM("Setup Visual Tools");

    //TODO: Use ns_ for the namespace
    // Load the Robot Viz Tools for publishing to rviz
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(base_link_,"/romeo_grasper/moveit_visual_tools"));

    visual_tools_->setPlanningSceneTopic("/move_group/monitored_planning_scene");
    //visual_tools_->loadPlanningSceneMonitor();
    visual_tools_->loadMarkerPub(true);
    //visual_tools_->loadRobotStatePub("display_robot_state");
    //visual_tools_->setManualSceneUpdating(); if use that must use // Send ROS messages visual_tools_->triggerPlanningSceneUpdate();

    // Allow time to publish messages
    ros::Duration(1.0).sleep();

    // Clear collision objects and markers
    ROS_DEBUG_STREAM("Cleaning collision objects and markers");
    visual_tools_->deleteAllMarkers();
    visual_tools_->removeAllCollisionObjects();
    //visual_tools_->triggerPlanningSceneUpdate();
    ros::Duration(0.1).sleep();

    visual_tools_->cleanupCO(SUPPORT_SURFACE3_NAME);

    // TODO: Test floor_to_base_height if it's correct
    // If you want to be more exactly you should take the distance in z between body and l_sole or r_sole
    double floor_to_base_height = -1;

    // I can't use it because floor_to_base_height in rviz_visual_tools is not used any more and then the table only can be put on z = 0
    //visual_tools_->publishCollisionTable(TABLE_X, TABLE_Y, 0, TABLE_WIDTH, TABLE_HEIGHT, TABLE_DEPTH, SUPPORT_SURFACE3_NAME, rviz_visual_tools::GREEN); // andy table
    std::vector< geometry_msgs::Point> points = tableMesuramentsToPoints(TABLE_X, TABLE_Y, TABLE_WIDTH, TABLE_HEIGHT, TABLE_DEPTH, floor_to_base_height);
    visual_tools_->publishCollisionCuboid(points.at(0),points.at(1), SUPPORT_SURFACE3_NAME, rviz_visual_tools::GREEN);

    //TODO: Use ns_ for the namespace
    string topic = "/romeo_grasper/visual_table";
    uint32_t queue_size = 10;
    visual_table_sub_ = node_handle_.subscribe(topic, queue_size, &RomeoGrasper::callbackVisualTable, this);

    if(verbose_)
        ROS_INFO_STREAM("Spawned collision table with name: " << SUPPORT_SURFACE3_NAME);

    is_visual_tools_setup_ = true;
}

void RomeoGrasper::setActionParams(moveit_simple_actions::Action* current_action)
{
    ROS_DEBUG_STREAM("Setup Action " << current_action->arm);
    current_action->setVerbose(verbose_);
    current_action->setMaxVelocityScalingFactor(max_velocity_scaling_factor_);
    current_action->setAttemptsMax(attempts_max_);
    current_action->setToleranceStep(tolerance_step_);
    current_action->setToleranceMin(tolerance_min_);
    current_action->setPlanningTime(planning_time_);
}

void RomeoGrasper::objectTrackerSetup()
{
    if(tracking_)
    {
        ROS_INFO("Configuring services with objectTracker node");
        change_model_service_client_ = node_handle_.serviceClient<object_tracker_srv_definitions::change_tracking_model>("/object_tracker/change_tracking_model");
    }

    ROS_INFO("Creating Subscribers to objectTracker...");
    string topic = "/object_tracker/object_pose";
    uint32_t queue_size = 10;
    obj_pose_sub_ = node_handle_.subscribe(topic, queue_size, &RomeoGrasper::callbackObjectPose, this);

    // With the new topic we don't need the topic object_tracker_confidence
    //topic = "/object_tracker/object_tracker_confidence";
    //tracker_confidence_sub_ = node_handle_.subscribe(topic, queue_size, &RomeoGrasper::callbackTrackerConfidence, this);
}

void RomeoGrasper::run()
{
    ROS_INFO("Running...");

    ros::Rate loop_rate(1);
    while(ros::ok())
    {
        loadParams(LOAD_PARAMS_RUN);

        if(modeled_object_->getHasObjectPose() && enough_confidence_ && changed_pose_ && !is_busy_)
        {
            planningAndExecutePoseGoal();
            ROS_INFO("Waiting for a new goal...");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void RomeoGrasper::planningAndExecutePoseGoal()
{
    //TODO: TEST!!!!!!!!!!!!!!!!!!!!!!
    // For now is not printing any object. So something is wrong

    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to deal directly with the world.
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ROS_INFO_STREAM("List of objects in the world:");
    std::vector< std::string > list_objects = planning_scene_interface.getKnownObjectNames();
    for(int i = 0; i < list_objects.size(); i++)
    {
        ROS_INFO_STREAM(list_objects[i]);
    }

    ROS_INFO("Start planning...");

    is_busy_ = true;
    moveit_simple_actions::Action* current_action = currentAction();

    if (verbose_)
        ROS_INFO_STREAM("Planning block with name " << modeled_object_->block_->name << " at pose " << modeled_object_->block_->start_pose);

    //TODO: Test if it takes the current state or not to make the plan
    //group.setStartState(*(group.getCurrentState()));

    bool success;

    //TODO: Test on picking
    //success = true;
    //preGraspVsPick = false;

    // TODO: Use a surface to avoid collision with table
    string surface_name_ = SUPPORT_SURFACE3_NAME;

    if(preGraspVsPick)
    {
        if (verbose_)
            ROS_INFO("Planing reaching pregrasp");

        // First made the plan
        //TODO: Decide which use
        if(reachVsGrasp_)
        {
            current_action->setFlag(FLAG_NO_MOVE);
            success = current_action->reachPregrasp(modeled_object_->block_->start_pose, surface_name_);
        }else{
            //TODO: Figure out if it's going to a pregrasping pose or in the grasp pose
            //It use the function setPoseTargets for all the grasps giving the pose of the last link
            //with pose grasp_pose of the message Grasp.
            //the problem is that is not possible to know which is the pose_target.

            //Trick to be able to use the experiment1.py
            ros::Publisher pub_obj_pose = node_handle_.advertise<geometry_msgs::PoseStamped>("/pose_target", 10);
            geometry_msgs::PoseStamped pose_target;
            pose_target.pose = modeled_object_->block_->start_pose;
            pose_target.header.stamp = ros::Time();
            pose_target.header.frame_id = base_link_;
            pub_obj_pose.publish(pose_target);

            success = current_action->graspPlan(modeled_object_->block_, surface_name_);
        }
    }else
        ROS_INFO("Prepared to do the picking");

    is_busy_ = false;

    // Automatic_execution allow to don't need to wait for the answer of the user
    // If the planning has been succeded, it will proceed with the execution
    if(automatic_execution_)
    {
        answer_service_ = ANSWER_SRV_MOVE;
    }else
    {
        answer_service_ = ANSWER_SRV_WAIT;
        waiting_service_ = true;

        ros::Rate loop_rate(1);
        string service_info = ns_ + "/execute_plan \n" +  ns_ + "/replan \n" + ns_ + "/abort_plan";
        ROS_INFO("Waiting for call of one of the followings service: \n%s", service_info.c_str());

        while(waiting_service_)
        {
            // Load params every time to can make changes and then replan or execute
            loadParams(LOAD_PARAMS_RUN);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    int num_points;
    double movement_time;
    ros::Duration duration;

    switch (answer_service_) {
    case ANSWER_SRV_MOVE:
        ROS_INFO("Starting movement...");

        // Get the movement time from the trajectory published by the Action
        num_points = trajectory_.joint_trajectory.points.size();
        duration = trajectory_.joint_trajectory.points[num_points-1].time_from_start;
        movement_time = duration.toSec();

        if(preGraspVsPick && success)
        {
            if (verbose_)
                ROS_INFO("Start pregrasping");
            if(!simulation_)
            {
                current_action->setFlag(FLAG_MOVE);
                current_action->poseHandOpen();
                success = current_action->executeAction();

                //Wait the movement time
                sleep(movement_time);

                ROS_INFO_STREAM("Pregrasping " << success ? "SUCCESS" : "FAILED");

                //TODO: This is done because the picking is not working
                current_action->poseHandClose();
            }else
            {
                //TODO: Implement open and close hand for simulation
                success = romeo_simulator_->executeTrajectory("/joint_trajectory");

                //Wait the movement time
                sleep(movement_time);
            }

            // Now restart function to do the picking
            preGraspVsPick = false;
            planningAndExecutePoseGoal();
        }else if(success)
        {
            if (verbose_)
                ROS_INFO("Start pick action");
            current_action->poseHandOpen();
            success = current_action->pickAction(modeled_object_->block_, surface_name_, attempts_max_, planning_time_, tolerance_min_);
            ROS_INFO_STREAM("Picking " << success ? "SUCCESS" : "FAILED");
        }else{
            ROS_INFO("Cannot execute plan if the plan has failed");
            break;
        }

        changed_pose_ = false;
        break;

    case ANSWER_SRV_REPLAN:
        planningAndExecutePoseGoal();
        break;

    case ANSWER_SRV_ABORT:
        ROS_INFO("Plan aborted");

        // If abort plan then you won't do anything till the object has another pose
        // If you want to work with the object in the same position then you should
        // use execute, plan or wait and change parameters in another terminal
        changed_pose_ = false;
        break;

    case ANSWER_SRV_WAIT:
        ROS_ERROR("Something wrong has passed because is imposible to have waiting_service_ false and answer_service_ with ANSWER_SRV_WAIT");
        break;

    default:
        ROS_ERROR("Problem with variable answer_service_");
        break;
    }
}

void RomeoGrasper::loadParam(string param_name, string *param, string default_value, int param_flag)
{
    string new_value;
    node_handle_.param(param_name, new_value, default_value);

    if(*param != new_value)
    {
        if(verbose_ && !firstSetup_)
            ROS_INFO("Parameter: %s has changed from: %s to %s",param_name.c_str(),param->c_str(),new_value.c_str());
        *param = new_value;
        changedParam(param_flag);
    }
}

void RomeoGrasper::loadParam(string param_name, int *param, int default_value, int param_flag)
{
    int new_value;
    node_handle_.param(param_name, new_value, default_value);

    if(*param != new_value)
    {
        if(verbose_ && !firstSetup_)
            ROS_INFO("Parameter: %s has changed from: %i to %i",param_name.c_str(),*param,new_value);
        *param = new_value;
        changedParam(param_flag);
    }
}

void RomeoGrasper::loadParam(string param_name, float *param, float default_value, int param_flag)
{
    float new_value;
    node_handle_.param(param_name, new_value, default_value);

    if(*param != new_value)
    {
        if(verbose_ && !firstSetup_)
            ROS_INFO("Parameter: %s has changed from: %f to %f",param_name.c_str(),*param,new_value);
        *param = new_value;
        changedParam(param_flag);
    }
}

void RomeoGrasper::loadParam(string param_name, bool *param, bool default_value, int param_flag)
{
    bool new_value;
    node_handle_.param(param_name, new_value, default_value);

    if(*param != new_value)
    {
        if(verbose_ && !firstSetup_)
        {
            string True_str = "True";
            string False_str = "False";
            string info = std::string("Parameter: ") + param_name + std::string(" has changed from: ") + (*param ? True_str : False_str) + std::string(" to ") + (new_value ? True_str : False_str);
            ROS_INFO("%s",info.c_str());
        }
        *param = new_value;
        changedParam(param_flag);
    }
}

// TODO: Test and try to implement
// Not working for now, we need to compute difference between current pose and goal pose
// and I think that the function getCurrentPose will stamped with the /base_link frame_id
void RomeoGrasper::changedParam(int param_flag)
{
    moveit_simple_actions::Action* current_action = currentAction();

    //Only is necessary to make additional actions with parameters that are reloaded
    switch(param_flag)
    {
    case PARAM_MOVE_GROUP:
        break;

    case PARAM_MODEL_OBJECT_NAME:
        // In the firstSetup is loaded by the findCameraReference function
        if(tracking_ && !firstSetup_ && camera_positioned_)
            changeTrackingModel(model_object_name_);
        break;
    case PARAM_MODELS_DIRECTORY:
    {
        // The model_directory must end with slash
        std::string::iterator it = models_directory_.end() - 1;
        if (*it != '/')
        {
            models_directory_.append("/");
        }
        // TODO: Test if it's correct, I'm not pretty sure about it
        // TODO: Test too if it's possible to call this before current_model_name is assigned
        if(tracking_ && !firstSetup_)
            changeTrackingModel(current_model_name_);
        break;
    }

    case PARAM_CONFIDENCE_THRESHOLD:
        // May be should have a var with the current conf and stop the movement in case
        // that there is no enough confidence?
        break;

    case PARAM_POSE_THRESHOLD:
        break;

    case PARAM_VERBOSE:
        if(!firstSetup_)
            current_action->setVerbose(verbose_);
        break;

    case PARAM_MAX_VELOCITY_SCALE_FACTOR:
        if(!firstSetup_)
            current_action->setMaxVelocityScalingFactor(max_velocity_scaling_factor_);
        break;

    case PARAM_ATTEMPTS_MAX:
        if(!firstSetup_)
            current_action->setAttemptsMax(attempts_max_);
        break;

    case PARAM_TOLERANCE_STEP:
        if(!firstSetup_)
            current_action->setToleranceStep(tolerance_step_);
        break;

    case PARAM_TOLERANCE_MIN:
        if(!firstSetup_)
            current_action->setToleranceMin(tolerance_min_);
        break;

    case PARAM_PLANNING_TIME:
        if(!firstSetup_)
            current_action->setPlanningTime(planning_time_);
        break;

    default:
        break;

    }
}

void RomeoGrasper::callbackTrajectory(moveit_msgs::RobotTrajectory data)
{
    trajectory_ = data;
}

void RomeoGrasper::callbackObjectPose(object_tracker_msg_definitions::ObjectInfo data)
{
    float x_data = data.translation.x;
    float y_data = data.translation.y;
    float z_data = data.translation.z;

    float x_pose = pose_target_stamped_.pose.position.x;
    float y_pose = pose_target_stamped_.pose.position.y;
    float z_pose = pose_target_stamped_.pose.position.z;

    ROS_DEBUG("Callback pose: %f,%f,%f with confidence of %f",x_data,y_data,z_data,data.confidence);

    enough_confidence_ = data.confidence >= confidence_threshold_;

    if(enough_confidence_ && (!has_pose_ || sqrt(pow(x_data-x_pose,2)+pow(y_data-y_pose,2)+pow(z_data-z_pose,2)) > pose_threshold_))
    {
        pose_target_stamped_.pose.position.x = x_data;
        pose_target_stamped_.pose.position.y = y_data;
        pose_target_stamped_.pose.position.z = z_data;

        pose_target_stamped_.pose.orientation.x = data.rotation.x;
        pose_target_stamped_.pose.orientation.y = data.rotation.y;
        pose_target_stamped_.pose.orientation.z = data.rotation.z;
        pose_target_stamped_.pose.orientation.w = data.rotation.w;

        pose_target_stamped_.header.frame_id = camera_frame_id_;
        pose_target_stamped_.header.stamp = ros::Time(0);

        modeled_object_->setNotObjectPose(); //The modeled object pose is not yet updated

        has_pose_ = true;
        changed_pose_ = true;
        preGraspVsPick = true;

        if(verbose_)
            ROS_INFO_STREAM("Pose object on frame " << camera_frame_id_ <<" found at:\n" << pose_target_stamped_.pose);
        //ROS_INFO("Pose object: %f,%f,%f with confidence of %f on frame: %s",pose_target_stamped_.pose.position.x,pose_target_stamped_.pose.position.y,pose_target_stamped_.pose.position.z,data.confidence, camera_frame_id_.c_str());

        if(camera_positioned_)
        {
            modeled_object_->updatePose(pose_target_stamped_);
            /*
            // Transform from base_link to camera
            tf::TransformListener tf_listener;
            geometry_msgs::PoseStamped pose_base_link;
            try{
                // Wait for Transform because in other case it continues with the running and use some
                // coords that it shouldn't
                ROS_DEBUG_STREAM("Transforming pose object from frame: " << pose_target_stamped_.header.frame_id << " to frame: " << base_link_);
                tf_listener.waitForTransform(base_link_, pose_target_stamped_.header.frame_id,
                                             ros::Time(0), ros::Duration(10.0));
                tf_listener.transformPose(base_link_, pose_target_stamped_, pose_base_link);
            }
            catch( tf::TransformException ex)
            {
                ROS_ERROR("Transfrom exception : %s",ex.what());
                ROS_ERROR("Pose object: %f,%f,%f and orientation: %f,%f,%f,%f",x_data,y_data,z_data,q.x(),q.y(),q.z(),q.w());
            }
            // Remove attached object
            visual_tools_->cleanupACO(block_->name);
            // Remove collision object
            visual_tools_->cleanupCO(block_->name);

            block_->updatePose(pose_base_link.pose);
            visual_tools_->publishCollisionCylinder(block_->start_pose, block_->name, block_->size, block_->size_l);
            visual_tools_->publishCylinder(block_->start_pose, rviz_visual_tools::RAND, block_->size_l, block_->size, block_->name);

            ros::Duration(1.0).sleep();
            if(verbose_)
                ROS_INFO("Pose object: %f,%f,%f with confidence of %f",pose_base_link.pose.position.x,pose_base_link.pose.position.y,pose_base_link.pose.position.z,data.confidence);
*/
        }
    }
}

void RomeoGrasper::callbackVisualTable(romeo_grasper::VisualTable data)
{
    visual_tools_->cleanupCO(SUPPORT_SURFACE3_NAME);
    std::vector< geometry_msgs::Point> points = tableMesuramentsToPoints(data.x, data.y, data.width, data.height, data.depth, data.floor_to_base_height);
    visual_tools_->publishCollisionCuboid(points.at(0),points.at(1), SUPPORT_SURFACE3_NAME, rviz_visual_tools::GREEN);
    ROS_INFO_STREAM("Visual Table updated");
    //TODO: Adapt roll and pitch to be the same as camera, assuming the camera is on a horitzontal surface
}

std::vector< geometry_msgs::Point> RomeoGrasper::tableMesuramentsToPoints(double x, double y, double width, double height, double depth, double floor_to_base_height)
{
    std::vector< geometry_msgs::Point> points;
    geometry_msgs::Point point1;
    geometry_msgs::Point point2;

    point1.x = x + depth/2.0;
    point1.y = y + width/2.0;
    point1.z = floor_to_base_height;
    point2.x = x - depth/2.0;
    point2.y = y - width/2.0;
    point2.z = floor_to_base_height + height;

    points.push_back(point1);
    points.push_back(point2);

    return points;
}

/* Not necessary with the new topic /object_tracker/object_pose
void RomeoGrasper::callbackTrackerConfidence(std_msgs::Float32 data)
{
    if(verbose_)
        ROS_INFO("Confidence of %f", data.data);

    enough_confidence_ = data.data >= confidence_threshold_;
}*/

//Only returns true if is waiting for the service, if is not don't do nothing and returns false
bool RomeoGrasper::executePlan(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
{
    if(waiting_service_)
    {
        waiting_service_ = false;
        answer_service_ = ANSWER_SRV_MOVE;
        resp.success = true;
    }else
    {
        resp.success = false;
        resp.message = "RomeoGrasper is not waiting for this service call right now";
    }
    return true;
}

bool RomeoGrasper::abortPlan(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
{
    if(waiting_service_)
    {
        waiting_service_ = false;
        answer_service_ = ANSWER_SRV_ABORT;
        resp.success = true;
    }else
    {
        resp.success = false;
        resp.message = "RomeoGrasper is not waiting for this service call right now";
    }
    return true;
}

bool RomeoGrasper::rePlan(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
{
    if(waiting_service_)
    {
        waiting_service_ = false;
        answer_service_ = ANSWER_SRV_REPLAN;
        resp.success = true;
    }else
    {
        resp.success = false;
        resp.message = "RomeoGrasper is not waiting for this service call right now";
    }
    return true;
}

void RomeoGrasper::publishTransforms()
{
    // publish transforms
    ROS_INFO_STREAM("RomeoGrasper - Publishing transforms");
    tf::TransformBroadcaster tf_broadcaster;

    ros::Duration sleeper(0.1); // 100ms

    while (ros::ok())
    {
        // TODO: Should I use the time_stamp or the stamp of the pose that I have???
        // time stamp is future dated to be valid for given duration
        ros::Time time_stamp = ros::Time::now() + sleeper;

        if(camera_positioned_ && modeled_object_->getHasObjectPose())
        {
            // Done by the class modeled_object
        }

        if(camera_positioned_)
        {
            tf::Transform tr;

            tr.setOrigin(tf::Vector3(camera_pose_.pose.position.x,
                                     camera_pose_.pose.position.y,
                                     camera_pose_.pose.position.z));

            tf::Quaternion q(camera_pose_.pose.orientation.x, camera_pose_.pose.orientation.y, camera_pose_.pose.orientation.z, camera_pose_.pose.orientation.w);
            tr.setRotation( q );
            ROS_DEBUG_STREAM("Sending transform child frame: " << camera_link_ << " with parent frame: " << base_link_);
            tf_broadcaster.sendTransform(tf::StampedTransform(tr, time_stamp, base_link_, camera_link_));

            // In case of not tracking we should publish too the camera_frame_id_ too
            // We suppose that is in the same position of camera_link but following the orientation of optical frame
            // According to REP 103
            // body -> (x - forward, y - left, z - up)
            // optical -> (z - forward, x - right, y - down)
            if(!tracking_)
            {
                tf::Transform tr_camera_frame;
                tr_camera_frame.setOrigin(tf::Vector3(0,0,0));

                tf::Quaternion q;
                q.setRPY(-M_PI/2, 0, -M_PI/2);
                tr_camera_frame.setRotation( q );
                //ROS_DEBUG_STREAM("Sending transform child frame: " << camera_frame_id_ << " with parent frame: " << camera_link_);
                tf_broadcaster.sendTransform(tf::StampedTransform(tr_camera_frame, time_stamp, camera_link_, camera_frame_id_));
            }
        }
        sleeper.sleep(); // need sleep or transform won't publish correctly
    }
}

void RomeoGrasper::findCameraReference()
{
    ROS_INFO_STREAM("Setup of Camera");
    tf::Quaternion q;

    // In case of knowing the exact poistion of the camera (camera_link)
    if(pose_camera_preknown_)
    {
        ROS_DEBUG_STREAM("Camera pose is preknown");
        if(!node_handle_.getParam("camera_pose_x", camera_pose_.pose.position.x) ||
                !node_handle_.getParam("camera_pose_y", camera_pose_.pose.position.y) ||
                !node_handle_.getParam("camera_pose_z", camera_pose_.pose.position.z))
        {
            //TODO: Use ns_ for the namespace
            // In case of pose_preknown say in the message that you need to put the info in the pertinent param
            // In case of tracking remember to initialise change_tracker_modeling service
            ROS_WARN_STREAM("Some of the following params doesn't exist:\n/romeo_grasper/camera_pose_x \n/romeo_grasper/camera_pose_y \n/romeo_grasper/camera_pose_z");
            if(!tracking_)
            {
                ROS_INFO_STREAM("Waiting for having these params or change to tracking mode");
                bool have_all_params = false;
                while(ros::ok && (!have_all_params || !tracking_))
                {
                    loadParam("tracking",
                              &tracking_,
                              false,
                              PARAM_TRACKING);

                    if(node_handle_.getParam("camera_pose_x", camera_pose_.pose.position.x) &&
                            node_handle_.getParam("camera_pose_y", camera_pose_.pose.position.y) &&
                            node_handle_.getParam("camera_pose_z", camera_pose_.pose.position.z))
                    {
                        have_all_params = true;
                    }
                }

                // If tracking start again but loading the change model service
                // and if have_all_params only continue with these new values
                if(tracking_)
                {
                    ROS_INFO("Changing to tracking mode");
                    ROS_INFO("Configuring services with objectTracker node");
                    change_model_service_client_ = node_handle_.serviceClient<object_tracker_srv_definitions::change_tracking_model>("/object_tracker/change_tracking_model");
                    findCameraReference();
                    return;
                }
            }else{
                ROS_WARN("Preknown camera pose failed, so starting positioning with tracking mode");
                pose_camera_preknown_ = false;
                findCameraReference();
                return;
            }
        }

        ROS_INFO_STREAM("Camera positioned at x:" << camera_pose_.pose.position.x
                        << " y:" << camera_pose_.pose.position.y
                        << " z:" << camera_pose_.pose.position.z
                        << " on reference at frame: " << camera_reference_frame_id_);

        // When is a different frame it only counts the position of the frame
        // the orientation will use the map orientation
        if(camera_reference_frame_id_ != base_link_)
        {
            // Transformation from base_link to the camera_link
            tf::StampedTransform transform;
            tf::TransformListener tf_listener;
            try{
                tf_listener.waitForTransform(base_link_, camera_reference_frame_id_,
                                             ros::Time(0), ros::Duration(10.0));
                ROS_DEBUG_STREAM("Look up for a transform between " << base_link_ << " and " <<  camera_reference_frame_id_);
                tf_listener.lookupTransform(base_link_, camera_reference_frame_id_,
                                            ros::Time(0), transform);
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }

            camera_pose_.pose.position.x = transform.getOrigin().x() + camera_pose_.pose.position.x;
            camera_pose_.pose.position.y = transform.getOrigin().y() + camera_pose_.pose.position.y;
            camera_pose_.pose.position.z = transform.getOrigin().z() + camera_pose_.pose.position.z;

            ROS_INFO_STREAM("Camera positioned at x:" << camera_pose_.pose.position.x
                            << " y:" << camera_pose_.pose.position.y
                            << " z:" << camera_pose_.pose.position.z
                            << " on reference at frame: " << base_link_);
        }

        float roll, pitch, yaw;
        if(node_handle_.getParam("camera_pose_roll", roll) &&
                node_handle_.getParam("camera_pose_pitch", pitch) &&
                node_handle_.getParam("camera_pose_yaw", yaw))
        {
            q.setRPY(roll, pitch, yaw);
        }
        else
        {
            // TODO: Some way to do it but robustly with orientations
            // Axis for cameras
            if(!camera_in_front_)
            {
                // When is next to the robot it means that is in an orientation perpendicular to the robot on the left side
                // Besides, the axis for a camera are different than for a robot
                // So with a Yaw of -90º we have the final orientation
                q.setEuler(0, 0, -M_PI/2);
            }else
            {
                // When is in front of the robot so X and Y axis are inverse from the map and Z is the same.
                // Besides, the axis for a camera are different than for a robot
                // TODO: Test this Quaternion // NO ENTENC AQUET TODO XD
                q.setEuler(0, 0, M_PI);
            }
        }

        camera_pose_.pose.orientation.x = q.x();
        camera_pose_.pose.orientation.y = q.y();
        camera_pose_.pose.orientation.z = q.z();
        camera_pose_.pose.orientation.w = q.w();

        if(tracking_)
            changeTrackingModel(model_object_name_);
    }else{

        if(tracking_)
        {
            ROS_DEBUG_STREAM("Look for a reference to pose the camera");
            // Confidence for find the reference should be high to don't have future problems
            float ref_confidence_threshold_;
            node_handle_.param("ref_confidence_threshold", ref_confidence_threshold_, 0.35f);
            confidence_threshold_ = ref_confidence_threshold_;

            changeTrackingModel(model_reference_name_);

            ros::Rate loop_rate(1);
            //int count = 0;
            // Wait to have a pose of reference to position the camera
            if(!has_pose_)
            {
                //TODO: Use ns_ for the namespace
                ROS_INFO_STREAM("Waiting to find the object that will be reference for the positioning of the camera");
                ROS_INFO_STREAM("Try to move the camera or the reference object with frame id: " << camera_reference_frame_id_ << " or to change the param /romeo_grasper/ref_confidence_threshold");
            }
            while(ros::ok && !has_pose_)
            {
                loadParam("ref_confidence_threshold",
                          &confidence_threshold_,
                          0.5f,
                          PARAM_REFERENCE_CONFIDENCE_THRESHOLD);

                ros::spinOnce();
                loop_rate.sleep();
                //                if(count == 5)
                //                {
                //                    ROS_INFO_STREAM("Waiting to find the object that will be reference for the positioning of the camera");
                //                }

                //                if (count == 10)
                //                {
                //                    ROS_INFO_STREAM("Waiting to find the object that will be reference for the positioning of the camera");
                //                    ROS_INFO_STREAM("Try to move the camera or the reference object with frame id: " << camera_reference_frame_id_);
                //                    count = 0;
                //                }
                //                count++;
            }

            // Apply offset of the reference model
            ROS_DEBUG_STREAM("Creating model of reference");
            ModeledObject *modeled_reference_ = new ModeledObject(&node_handle_,
                                                                  visual_tools_,
                                                                  model_reference_name_,
                                                                  "modeled_reference",
                                                                  pose_target_stamped_,
                                                                  true,
                                                                  false,
                                                                  verbose_);
            modeled_reference_->updatePose(pose_target_stamped_);

            if(ros::ok && verbose_)
            {
                ROS_INFO_STREAM("Position of reference founded");
                ROS_INFO_STREAM("Pose of reference " << camera_reference_frame_id_ << " on frame " << camera_frame_id_ <<" found at:\n" << modeled_reference_->block_->start_pose);
                //ROS_INFO("Pose of reference: %s found at: %f,%f,%f on frame: %s",camera_reference_frame_id_.c_str(), modeled_reference_->block_->start_pose.position.x,modeled_reference_->block_->start_pose.position.y,modeled_reference_->block_->start_pose.position.z, camera_frame_id_.c_str());
            }

            tf::StampedTransform tf_base_reference;
            tf::StampedTransform tf_frame_link;
            tf::TransformListener tf_listener;
            try{

                // Transformation from base_link to the reference
                tf_listener.waitForTransform(base_link_, camera_reference_frame_id_,
                                             ros::Time(0), ros::Duration(10.0));
                ROS_DEBUG_STREAM("Look up for a transform between " << base_link_ << " and " <<  camera_reference_frame_id_);
                tf_listener.lookupTransform(base_link_, camera_reference_frame_id_,
                                            ros::Time(0), tf_base_reference);

                // Transform from camera_frame to camera_link
                tf_listener.waitForTransform(camera_frame_id_, camera_link_,
                                             ros::Time(0), ros::Duration(10.0));
                ROS_DEBUG_STREAM("Look up for a transform between " << camera_frame_id_ << " and " <<  camera_link_);
                tf_listener.lookupTransform(camera_frame_id_, camera_link_,
                                            ros::Time(0), tf_frame_link);
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }

            // TODO: Some way to do it but robustly with orientations
            // According to REP 103
            // body -> (x - forward, y - left, z - up)
            // optical -> (z - forward, x - right, y - down)
            if(!camera_in_front_)
            {
                ROS_DEBUG_STREAM("Positioning camera on the left side");
                // When is next to the robot it means that is in an orientation perpendicular to the robot on the left side
                // Besides, the axis for a camera are different than for a robot
                camera_pose_.pose.position.x = tf_base_reference.getOrigin().x() + modeled_reference_->block_->start_pose.position.x;
                camera_pose_.pose.position.y = tf_base_reference.getOrigin().y() + modeled_reference_->block_->start_pose.position.z;
                camera_pose_.pose.position.z = tf_base_reference.getOrigin().z() + modeled_reference_->block_->start_pose.position.y;

                // When is next to the robot it means that is in an orientation perpendicular to the robot on the left side
                // Besides, the axis for a camera are different than for a robot
                // So with a Yaw of -90º we have the final orientation
                q.setEuler(0, 0, -M_PI/2);
            }else
            {
                ROS_DEBUG_STREAM("Positioning camera in front");
                // When is in front of the robot so X and Y axis are inverse from the map and Z is the same.
                // Besides, the axis for a camera are different than for a robot
                camera_pose_.pose.position.x = tf_base_reference.getOrigin().x() + modeled_reference_->block_->start_pose.position.z;
                camera_pose_.pose.position.y = tf_base_reference.getOrigin().y() - modeled_reference_->block_->start_pose.position.x;
                camera_pose_.pose.position.z = tf_base_reference.getOrigin().z() + modeled_reference_->block_->start_pose.position.y;

                // When is in front of the robot so X and Y axis are inverse from the map and Z is the same.
                // Besides, the axis for a camera are different than for a robot
                // TODO: Test this Quaternion // NO ENTENC AQUET TODO XD
                q.setEuler(0, 0, M_PI);
            }

            // Apply transform from camera_frame to camera_link
            // It depends on the axis for the optical frame
            camera_pose_.pose.position.x -= tf_frame_link.getOrigin().z();
            camera_pose_.pose.position.y += tf_frame_link.getOrigin().x();
            camera_pose_.pose.position.z += tf_frame_link.getOrigin().y();

            camera_pose_.pose.orientation.x = q.x();
            camera_pose_.pose.orientation.y = q.y();
            camera_pose_.pose.orientation.z = q.z();
            camera_pose_.pose.orientation.w = q.w();

            if(verbose_)
                ROS_INFO_STREAM("Pose camera on frame " << base_link_ <<" found at:\n" << camera_pose_.pose);
            //ROS_INFO("Pose camera: %f,%f,%f on frame: %s",camera_pose_.pose.position.x,camera_pose_.pose.position.y,camera_pose_.pose.position.z, base_link_.c_str());

            camera_pose_.header.stamp = pose_target_stamped_.header.stamp;
            camera_pose_.header.frame_id = base_link_;

            // Changing the model of the object and use the normal confidence threshold
            delete modeled_reference_;
            node_handle_.param("confidence_threshold", confidence_threshold_, 0.25f);
            changeTrackingModel(model_object_name_);
        }else
        {
            //TODO: Use ns_ for the namespace
            // In case of pose_preknown say in the message that you need to put the info in the pertinent param
            // In case of tracking remember to initialise change_tracker_modeling service
            ROS_WARN_STREAM("If is not tracking I need to know where is the camera positioned");
            ROS_WARN_STREAM("Waiting for setting mode pose_camera_preknown or tracking by one of these params:\n/romeo_grasper/pose_camera_preknown \n/romeo_grasper/tracking");
            while(ros::ok && (!pose_camera_preknown_ || !tracking_))
            {
                loadParam("tracking",
                          &tracking_,
                          false,
                          PARAM_TRACKING);

                loadParam("pose_camera_preknown",
                          &pose_camera_preknown_,
                          false,
                          PARAM_POSE_CAMERA_PREKNOWN);
            }

            if(tracking_)
            {
                // Starts again but loading the change model service
                ROS_INFO("Configuring services with objectTracker node");
                change_model_service_client_ = node_handle_.serviceClient<object_tracker_srv_definitions::change_tracking_model>("/object_tracker/change_tracking_model");
                findCameraReference();
                return;
            }else if(pose_camera_preknown_)
                findCameraReference();
                return;
            return;
        }
    }

    camera_pose_.header.frame_id = base_link_;
    camera_pose_.header.stamp = ros::Time::now();

    ROS_DEBUG_STREAM("Spawning cuboid of camera");
    if(!camera_in_front_)
        visual_tools_->publishCuboid(camera_pose_.pose, 0.1, 0.2, 0.1);
    else
        visual_tools_->publishCuboid(camera_pose_.pose, 0.2, 0.1, 0.1);

    ROS_INFO_STREAM("Camera positioned, do NOT move the camera");
    camera_positioned_ = true;
    modeled_object_->setCameraPositioned(true);
}

void RomeoGrasper::changeTrackingModel(string model_name)
{
    // Only change the tracking model if the service exists in other case will remain here till that moment
    // If this service doesn't exists it means that the camera hasn't been recognised so we can't continue
    if(!change_model_service_client_.exists())
    {
        change_model_service_client_.waitForExistence();
    }

    object_tracker_srv_definitions::change_tracking_model srv_change;
    srv_change.request.filename = models_directory_ + model_name + "/tracking_model.ao";
    if(change_model_service_client_.call(srv_change))
    {
        ros::ServiceClient start_tracking_service_client = node_handle_.serviceClient<std_srvs::Empty>("/object_tracker/start_recording");

        if(!start_tracking_service_client.exists())
        {
            if(!change_model_service_client_.waitForExistence(ros::Duration(1)))
            {
                ROS_INFO_STREAM("Couldn't load model from file: " << srv_change.request.filename);
                return;
            }
        }

        std_srvs::Empty srv_start;
        if(start_tracking_service_client.call(srv_start))
        {
            ROS_INFO_STREAM("Load model from file: " << srv_change.request.filename);
            current_model_name_ = model_name;
            if(verbose_)
                ROS_INFO_STREAM("Object tracker restarted with service /object_tracker/start_recording");
        }
    }
    // With new model the pose we had is not useful and we should wait for a new one
    has_pose_ = false;
    if(model_name.compare(modeled_object_->getModelName()) == 0)
        modeled_object_->setNotObjectPose();
}

void RomeoGrasper::exit()
{
    //TODO: Use smart pointers to a better performance
    ROS_INFO("Exit RomeoGraspingObject");
    delete action_right_;
    delete action_left_;
    //delete block_;

    ros::ServiceClient stop_tracking_service_client = node_handle_.serviceClient<std_srvs::Trigger>("/object_tracker/stop_recording");
    std_srvs::Trigger srv;
    stop_tracking_service_client.call(srv);

    ros::ServiceClient cleanup_service_client = node_handle_.serviceClient<std_srvs::Trigger>("/object_tracker/cleanup");
    cleanup_service_client.call(srv);

    visual_tools_->deleteAllMarkers();
    visual_tools_->removeAllCollisionObjects();

    ros::shutdown();
}

moveit_simple_actions::Action* RomeoGrasper::currentAction()
{
    bool right_arm;
    string right_str = "right";
    std::size_t found = move_group_.find(right_str);
    right_arm = found!=std::string::npos;

    if(right_arm)
        return action_right_;
    else
        return action_left_;
}

/*
bool RomeoGrasper::naoqiSetup()
{
    string broker_name = "Romeo Grasper Object Broker";
    int pport;
    string pip;
    int broker_port;
    string broker_ip;

    // Load Params from Parameter Server
    node_handle_.param("RobotIP", pip, string("127.0.0.1"));
    node_handle_.param("RobotPort", pport,9559);
    node_handle_.param("DriverBrokerPort", broker_port, 54000);
    node_handle_.param("DriverBrokerIP", broker_ip, string("0.0.0.0"));

    ROS_INFO("pport %i, pip %s, broker_port %i, broker_ip %s", pport,pip.c_str(),broker_port,broker_ip.c_str());

    try
    {
        broker_ = AL::ALBroker::createBroker(broker_name,broker_ip,broker_port,pip,pport,0);

        try
        {
           motionProxy_ = boost::shared_ptr<AL::ALMotionProxy>(new AL::ALMotionProxy(broker_));
           ROS_INFO("Motion Proxy created successfully");
           return true;
        }
        catch (const AL::ALError& e)
        {
           ROS_ERROR("Could not create ALMotionProxy.");
           return false;
        }
    }

    catch(...)
    {
        ROS_ERROR("Failed to connect to Broker at %s:%d!",pip.c_str(),pport);
        return false;
    }
}

void RomeoGrasper::headStiffness(float movement_time)
{
    float stiffnessList[] = {0.6f, 0.6f, 0.0f};
    float timeList[] = {1.0f, movement_time, movement_time + 1.0f};
    motionProxy_->stiffnessInterpolation("Head", stiffnessList, timeList);
}
*/
