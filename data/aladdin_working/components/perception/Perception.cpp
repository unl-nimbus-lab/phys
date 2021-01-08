/*
 * Controller.cpp
 *
 *  Created on: 29-okt-2008
 *      Author: sspr
 */


#include "Perception.hpp"
#include <rtt/Component.hpp>

using namespace RTT;

namespace UseCase
{
	
	
		Perception::Perception(const std::string& name) :
			TaskContext(name, PreOperational)
		{
			this->provides()->addProperty("table_robot_transform", table_robot_transform).doc("table robot transform");
			this->provides()->addProperty("robot_table_transform", robot_table_transform).doc("robot table transform");
			this->provides()->addProperty("isDesiredPoseSet", is_desired_pose_set).doc("bool variable, true if desired pose is set correctly");
			this->provides()->addProperty("actionType", action_type).doc("integer showing the type of action");
			this->provides()->addProperty("enable_update_relation", enable_update_relation).doc("enable_update_relation");
			this->provides()->addProperty("toolRobotPose", tool_actual_robot).doc("geometry_msgs::Pose of tool in robot frame");
			this->provides()->addProperty("tool_actual_table", tool_actual_table).doc("geometry_msgs::Pose of tool in table frame");
// 			this->provides()->addProperty("obj1PoseMsgRobot", obj1_msgs_robot).doc("geometry_msgs::Pose of obj1 in robot frame");
			this->provides()->addProperty("obj1PoseMsgCamera", obj1_camera).doc("geometry_msgs::Pose of obj1 in camera frame");
			this->provides()->addProperty("obj1PoseMsgTable", obj1_table).doc("geometry_msgs::Pose of obj1 in table frame");
// 			this->provides()->addProperty("obj2PoseMsgRobot", obj2_msgs_robot).doc("geometry_msgs::Pose of obj2 in robot frame");
			this->provides()->addProperty("obj2PoseMsgCamera", obj2_camera).doc("geometry_msgs::Pose of obj2 in camera frame");
			this->provides()->addProperty("obj2PoseMsgTable", obj2_table).doc("geometry_msgs::Pose of obj2 in table frame");
			this->provides()->addProperty("GoalPose", goal_table).doc("geometry_msgs::Pose Goal");
			this->provides()->addProperty("CalibPose1", calibpose1).doc("geometry_msgs::Pose Calibration Pose 1");
			this->provides()->addProperty("CalibPose2", calibpose2).doc("geometry_msgs::Pose Calibration Pose 2");
			this->provides()->addProperty("HomePose", home_table).doc("geometry_msgs::Pose Home");
			this->provides()->addProperty("DesiredPose", arm_desired_robot).doc("geometry_msgs::Pose Desired arm pose");
			//this->provides()->addProperty("isObjSet", is_obj_set).doc("true if the Object pose is set");
			this->provides()->addProperty("actualRelations", actual_relations).doc("relation between objects (a vector of ints with 6 elements)");
			this->provides()->addProperty("MsrWrench", msr_wrench).doc("measured Wrench via FRI");
			this->provides()->addProperty("handStatus", hand_status_int).doc("status of hand");
			this->provides()->addProperty("actionTypeIsSet", action_type_is_set).doc("action type is received by perception");
			this->provides()->addProperty("obj1PoseIsSet", obj1_pose_is_set).doc("obj1 pose is received by perception");
			this->provides()->addProperty("obj2PoseIsSet", obj2_pose_is_set).doc("obj2 pose is received by perception");
			this->provides()->addProperty("isPerceptionReady", is_perception_ready).doc("perception is ready");
			this->provides()->addProperty("armMsgs", arm_actual_robot).doc("pose of arm");
			this->provides()->addProperty("isRelation1InitReceived", is_relation_1_init_received).doc("flag");
			this->provides()->addProperty("isRelation2InitReceived", is_relation_2_init_received).doc("flag");
			this->provides()->addProperty("isRelation3InitReceived", is_relation_3_init_received).doc("flag");
			this->provides()->addProperty("relation1Init", relation_1_init).doc("initial value of relation 1");
			this->provides()->addProperty("relation2Init", relation_2_init).doc("initial value of relation 2");
			this->provides()->addProperty("relation3Init", relation_3_init).doc("initial value of relation 3");
			this->provides()->addProperty("isObj1Round", is_obj1_round).doc("true if the obj1 is round, false if elongated");
			this->provides()->addProperty("obj1Angle", obj1_angle).doc("angle of obj1 wrt X axis");
			this->provides()->addProperty("obj2Angle", obj2_angle).doc("angle of obj2 wrt X axis");
			this->provides()->addProperty("knifeAngleCosine", knife_angle_cosine).doc("cosine of knife angle");
			this->provides()->addProperty("knifeAngleSine", knife_angle_sine).doc("sine of knife angle");
			this->provides()->addProperty("is_planner_enabled", is_planner_enabled).doc("is planner enabled?");
			this->provides()->addProperty("is_vision_enabled", is_vision_enabled).doc("is vision enabled?");
			this->provides()->addProperty("saw_vector_robot", saw_vector_robot).doc("vector of sawing movement in robot frame");
			this->provides()->addProperty("stir_vector_robot", stir_vector_robot).doc("vector of stirring movement in robot frame");
			this->provides()->addProperty("armNumber", arm_number).doc("arm number: -1=undefined, 0=right arm, 1=left arm");
			/// input ports
			this->provides()->addEventPort( "obj1PoseInport", obj1_pose_inport ).doc("reads the pose of obj1");
			this->provides()->addEventPort( "obj2PoseInport", obj2_pose_inport ).doc("reads the pose of obj2");
			this->provides()->addEventPort( "goalPoseInport", goal_pose_inport ).doc("reads the goal pose");
			this->provides()->addEventPort( "ArmPoseInport", arm_actual_inport ).doc("reads the pose of tool from FRI");
			this->provides()->addEventPort( "relationsVision", relations_vision_inport ).doc("reads relations from vision");
			this->provides()->addEventPort( "WrenchPort", wrench_port ).doc("measured wrench at robot TCP via FRI");
			this->provides()->addEventPort( "HandStatusInport", hand_status_inport ).doc("status of hand");
			this->provides()->addEventPort( "HandStatusInport2", hand_status_inport2 ).doc("status of hand2");
			this->provides()->addEventPort( "ArmStatusInport", arm_status_inport ).doc("status of arm");
			this->provides()->addEventPort( "actionTypeInport", action_type_inport ).doc("reads the type of action");
			this->provides()->addEventPort( "toolTypeInport", tool_type_inport ).doc("reads the tool of action");
			this->provides()->addEventPort( "obj1AngleInport", obj1_angle_inport ).doc("reads obj1 angle");
			this->provides()->addEventPort( "relationPropertiesInport", relation_properties_inport ).doc("reads the relation properties");
			this->provides()->addEventPort( "initRelation1Inport", init_relation1_inport ).doc("reads the initial relation 1");
			this->provides()->addEventPort( "initRelation2Inport", init_relation2_inport ).doc("reads the initial relation 2");
			this->provides()->addEventPort( "initRelation3Inport", init_relation3_inport ).doc("reads the initial relation 3");
			this->provides()->addEventPort( "freeDirectionInport", freeDirectionInport ).doc("reads the free direction for movement");
			/// output ports
// 			this->provides()->addPort( "allPose", all_pose_outport ).doc("writes the pose of all objects");
			this->provides()->addPort( "ActualRelationsOutport", relations_outport ).doc("writes all the relation");
			this->provides()->addPort( "relationTypeIntMsgsOutport", relation_type_int_msgs_outport ).doc("writes type of relations");
			this->provides()->addPort( "relationSensorIntMsgsOutport", relation_sensor_int_msgs_outport ).doc("writes sensor of relations");
			this->provides()->addPort( "armGoalMsgsOutport", arm_goal_msgs_outport ).doc("writes the next arm goal to ROS");
// 			this->provides()->addPort( "Target", target ).doc("Target signal");
// 			this->provides()->addPort( "Sense", sense ).doc("Measurement signal");
			this->provides()->addPort( "ObjSetOutPort", obj_set_outport ).doc("output port that writes integer 1 when object pose is set");
			this->provides()->addPort( "filteredWrenchOutport", filtered_wrench_port ).doc("writes the filtered wrench in table frame");
			this->provides()->addPort( "rawWrenchOutport", raw_wrench_port ).doc("writes the raw wrench in table frame");
			this->provides()->addPort( "wrenchVelocityOutport", wrench_velocity_port ).doc("writes the filtered wrench velocity");
			this->provides()->addPort( "contactDetectedOutport", contact_detected_outport ).doc("writes the detected contact in X,Y,Z axes");
			this->provides()->addPort( "desiredPoseOutport", desired_pose_outport ).doc("writes the desired pose of tool in table frame ");
			this->provides()->addPort( "desiredPoseArmOutport", desired_pose_arm_outport ).doc("writes the desired pose of arm in table frame ");
			this->provides()->addPort( "desiredPoseArmRobotOutport", desired_pose_arm_robot_outport ).doc("writes the desired pose of arm in robot frame ");
			this->provides()->addPort( "actualToolPoseOutport", actual_tool_pose_outport ).doc("writes the actual pose of tool in table frame ");
			this->provides()->addPort( "actualArmPoseOutport", actual_arm_pose_outport ).doc("writes the actual pose of arm in table frame ");
			this->provides()->addPort( "opposing_force_outport", opposing_force_outport ).doc("writes the amplitude of force opposite of moving direction ");
// 			this->provides()->addPort( "toolMsgsOutport", tool_robot_outport ).doc("writes the actual tool pose ");
			this->provides()->addPort( "actualRelationsIntMsgsOutport", actual_relations_int_msgs_outport ).doc("writes the actual relations int msgs ");
			this->provides()->addPort( "isObj1RoundMsgsOutport", is_obj1_round_msgs_outport ).doc("writes the boolean msgs bool is_obj1_round_msgs_outport ");
			this->provides()->addPort( "isObj2RoundMsgsOutport", is_obj2_round_msgs_outport ).doc("writes the boolean msgs bool is_obj2_round_msgs_outport ");
			/// Operations 
			this->addOperation( "setPose", &Perception::set_pose, this ).doc("set the obj pose");
			this->addOperation( "calculateTime", &Perception::calculate_time, this ).doc("calculate and return the time for the next trajectory");
			this->addOperation( "initRelations", &Perception::initialize_relations, this ).doc("initialize the value of actual relations");
			this->addOperation( "setDesiredPose", &Perception::set_desired_pose, this ).doc("set the next desired pose for robot arm");
			this->addOperation( "isArmAboveDesired", &Perception::is_arm_above_desired, this ).doc("returns true if arm is above desired point");
			this->addOperation( "setCurrentArmGoal", &Perception::set_current_arm_goal, this ).doc("sets the current arm goal");
			this->addOperation( "saveRelations", &Perception::save_relations, this ).doc("save the actual_relations");
			this->addOperation( "loadRelations", &Perception::load_relations, this ).doc("load the actual_relations");
			this->addOperation( "printArmMsgs", &Perception::print_arm_pose, this ).doc("print arm pose in robot frame");
			this->addOperation( "final_pose", &Perception::final_pose, this ).doc("generates the final goal pose");
			this->addOperation( "calculate_saw_motion", &Perception::calculate_saw_motion, this ).doc("calculate the saw motion for cutting");
			this->addOperation( "calculate_stir_motion", &Perception::calculate_stir_motion, this ).doc("calculate the stir motion for stirring");
			this->addOperation( "set_tool_offset", &Perception::set_tool_offset, this ).doc("set the tool offset");
			this->addOperation( "set_cutting_point", &Perception::set_cutting_point, this ).doc("set the cutting point for cutting action");
			this->addOperation( "set_cutting_sidepoint", &Perception::set_cutting_sidepoint, this ).doc("set_cutting_sidepoint");
			this->addOperation( "move_main2", &Perception::move_main2, this ).doc("moves main object 2.");
			this->addOperation( "move_main3", &Perception::move_main3, this ).doc("moves main object 3.");
// 			this->addOperation( "update_relation", &Perception::update_relation, this ).doc("update_relation");
			this->addOperation( "detect_tool_background_relation", &Perception::detect_tool_background_relation, this ).doc("detect_tool_background_relation");
			arm_status_string = "robot_at_init";
			hand_status_string = "undefined";
			hand_status = HAND_UNDEFINED;
			arm_status = ARM_AT_INIT;
			is_desired_pose_set = false;
			current_goal_point= NO_ARG;
			action_type=0; /// default value, corresponds to no action
			obj1_pose_is_set=obj2_pose_is_set=tool_type_is_set=action_type_is_set= is_arm_actual_received=relation_properties_is_set=false;
			obj1_angle_is_set = false;
			obj2_angle_is_set = false;
			is_perception_ready = false;
			force_filter_is_set = false;
			filter_alpha = 0.05;
			is_obj1_round = true;
			is_obj1_round_msgs.data = is_obj1_round;
			is_planner_enabled = false;
			is_vision_enabled = false;
			arm_number = -1; // 0= right arm, 1= left arm, -1= undefined
			is_free_direction_set = false;
			second_putontop = false;
			
		}

		bool Perception::configureHook()
		{
// 			std::vector <geometry_msgs::Pose> example(10);
// 			all_pose_outport.setDataSample(example);
		  
		  /// initialize the tool to SDH hand only
		  //log(Warning)<<"perception::configureHook >> tool_offset is set to hand only"<<endlog();
		  set_tool_offset(TOOL_HAND);
// 		  tool_offset.data[0] = 0;
// 		  tool_offset.data[1] = 0;
// 		  tool_offset.data[2] = 0.26;
		  tool_type_is_set = true;
		  filtered_wrench_initialized = false;
		  tool_type = NO_TOOL;
		  is_relation_1_init_received = false;
		  is_relation_2_init_received = false;
		  is_relation_3_init_received = false;
		  /// set the camera_table_transform
		  camera_table_transform.p.x(0.054269) ;
		  camera_table_transform.p.y(1.3909) ;
		  camera_table_transform.p.z(0.50315) ;
		  KDL::Vector rot_x(-9.99855161e-01, -1.57603379e-02, -6.43653795e-03);
		  KDL::Vector rot_y(-3.18160281e-03, 5.44415772e-01, -8.38809550e-01);
		  KDL::Vector rot_z(1.67240743e-02, -8.38667572e-01, -5.44386923e-01);
		  KDL::Rotation rot_matrix (rot_x,rot_y,rot_z);
		  camera_table_transform.M = rot_matrix;
		  /// setting the robot-table transform
		  float temp_alpha,temp_beta,temp_gamma;
		  //log(Warning)<<"perception::configureHook >> arm number is= "<<arm_number<<endlog();
		  if(arm_number==1) /// left arm : arm_number =1
		  {
		    /// position
		    table_robot_transform.p.x(-0.09) ;
		    table_robot_transform.p.y(0) ;
		    //table_robot_transform.p.z(0.47) ;
		    table_robot_transform.p.z(0.32) ;
		    /// orientation -49.1066 -48.5904 40.8934
		    temp_alpha = -49.1066*PI/180;
		    temp_beta = -48.5904*PI/180;
		    temp_gamma = 40.8934*PI/180;
		  }
		  else if(arm_number==0) /// right arm : arm_number =0
		  {
		    /// position
		    table_robot_transform.p.x(0.09) ;
		    table_robot_transform.p.y(0) ;
		    //table_robot_transform.p.z(0.47) ;
		    table_robot_transform.p.z(0.32) ;
		    /// orientation -49.1066 -48.5904 40.8934
		    temp_alpha = -49.1066*PI/180;
		    temp_beta = 48.5904*PI/180;
		    temp_gamma = -40.8934*PI/180;
		  }
		  else
		  {
		    log(Error)<<"perception::configureHook >> invalid arm number="<<arm_number<<endlog();
		    return false;
		  }
		  KDL::Rotation temp_rot = KDL::Rotation::EulerZYX(temp_alpha,temp_beta,temp_gamma);
		  //log(Warning)<<"Rotation matrix: unit X= "<<temp_rot.UnitX().data[0]<<" "<<temp_rot.UnitX().data[1]<<" "<<temp_rot.UnitX().data[2]<<endlog();
		  //log(Warning)<<"Rotation matrix: unit Y= "<<temp_rot.UnitY().data[0]<<" "<<temp_rot.UnitY().data[1]<<" "<<temp_rot.UnitY().data[2]<<endlog();
		  //log(Warning)<<"Rotation matrix: unit Z= "<<temp_rot.UnitZ().data[0]<<" "<<temp_rot.UnitZ().data[1]<<" "<<temp_rot.UnitZ().data[2]<<endlog();
		  table_robot_transform.M = temp_rot;
		  robot_table_transform = table_robot_transform.Inverse();
		  geometry_msgs::Transform table_robot_transform_msgs;
		  tf::transformKDLToMsg(table_robot_transform,table_robot_transform_msgs);
// 		  log(Error)<<" ************ Transforms ******************"<<endlog();
// 		  log(Error)<<table_robot_transform_msgs<<endlog();
		  /// set CalibPose1 and CalibPose2
		  //geometry_msgs::Vector3 pos1;
		  
		  /// pose 1
		  //KDL::Vector p1(-0.195,0.33776,0.63247); // right arm  (TCP at kuka arm tip)
		  KDL::Vector p1(-0.4,0.4,0.4); // right arm  (TCP at sensor tip)
		  //KDL::Vector p1(0.195,-0.33776,0.567002); // left arm (TCP is at sensor tip)
		  double alpha1 = 120*PI/180;
		  double beta1 = 0*PI/180;
		  double gamma1 = -180*PI/180;
		  KDL::Rotation r1 = KDL::Rotation::EulerZYX(alpha1,beta1,gamma1);
		  /// pose 2
// 		  KDL::Vector p2(-0.267,0.461468,0.71038); // right arm
// 		  //KDL::Vector p2(0.266523,-0.461618,0.709907); // left arm
// 		  double alpha2 = 86*PI/180;
// 		  //double beta2 = 90*PI/180; /// left arm
// 		  double beta2 = -90*PI/180; /// right arm
// 		  //double gamma2 = -160*PI/180; /// left arm
// 		  double gamma2 = -145*PI/180; /// right arm
// 		  KDL::Rotation r2 = KDL::Rotation::EulerZYX(alpha2,beta2,gamma2);
		  //KDL::Rotation r1 = KDL::Rotation::EulerZYX(alpha,beta,gamma);
// 		  frame1.M.EulerZYX(alpha1,beta1,gamma1);
		  KDL::Frame frame_table;
		  frame_table.p = p1;
		  frame_table.M = r1;
		  geometry_msgs::Pose pose_table;
		  tf::poseKDLToMsg(frame_table,pose_table);
		  transform_table_to_robot(pose_table,calibpose1);
		  
		  calibpose2 = calibpose1;
		  enable_update_relation = true;
		  update_positions = true;
		  return true;
		}

		bool Perception::startHook()
		{
		  //log(Warning)<<"Perception:startHook >> started ..."<<endlog();
		  if(!force_filter_is_set)
		  {
		    geometry_msgs::Wrench wrench;
		    if(wrench_port.read(wrench)==NoData)
		    {
		      log(Error) << this->getName() << " cannot start if " <<wrench_port.getName()<<" has no input data."<<endlog();
		      //filtered_wrench = msr_wrench;
		      return false;
		    }
		    else
		    {
		      force_filter_is_set = true;
		    }
		  }

		  if(!is_arm_actual_received)
		  {
		    geometry_msgs::Pose arm_pose;
		    if(arm_actual_inport.read(arm_pose)==NoData)
		    {
		      log(Error) << this->getName() << " cannot start if " <<arm_actual_inport.getName()<<" has no input data."<<endlog();
		      return false;
		    }
		    else
		    {
		      float quaternion_norm = sqrt(arm_pose.orientation.x*arm_pose.orientation.x +arm_pose.orientation.y*arm_pose.orientation.y 
			+ arm_pose.orientation.z*arm_pose.orientation.z +arm_pose.orientation.w*arm_pose.orientation.w);
// 		      update_positions = true;
		      if(std::fabs(quaternion_norm-1) > 0.1)
		      {
			//log(Error)<<"Perception:startHook >> Quaternion norm= "<<quaternion_norm<<endlog();
			return false;
		      }
		      //log(Info)<<"Perception:startHook >> Quaternion norm= "<<quaternion_norm<<endlog();
		      arm_actual_robot = arm_pose;
		      //log(Warning)<<"perception::startHook >> Actual arm pose in robot frame\n"<<arm_actual_robot<<endlog();
		      transform_robot_to_table(arm_actual_robot,arm_actual_table);
		      //log(Warning)<<"perception::startHook >> Actual arm pose in table frame\n"<<arm_actual_table<<endlog();
		      /// add the tool_offset to the arm_actual_robot, to get tool_actual_robot (TCP)
		      adjust_tool(arm_actual_robot,tool_actual_robot,tool_offset,1);
		      adjust_tool(arm_actual_table,tool_actual_table,tool_offset,1);
		      log(Warning)<<"perception::startHook >> Actual tool pose in table frame=\n"<<tool_actual_table<<endlog();
		      actual_tool_pose_outport.write(tool_actual_table);
		      actual_arm_pose_outport.write(arm_actual_table);
// 		      KDL::Frame frame1;
// 		      tf::poseMsgToKDL(tool_actual_table,frame1);
// 		      double angle_alfa,angle_beta,angle_gamma;
// 		      frame1.M.GetEulerZYX(angle_alfa,angle_beta,angle_gamma);
// 		      //log(Warning)<<"perception::startHook >> Euler angles are=\n"<<angle_alfa*180/PI<<"\n"<<angle_beta*180/PI<<"\n"<<angle_gamma*180/PI<<endlog();
		      std::vector<double> euler_angles = getZYXEulerfromPose(tool_actual_table);
		      //log(Warning)<<"perception::startHook >> Euler angles are=\n"<<euler_angles.at(0)<<" "<<euler_angles.at(1)<<" "<<euler_angles.at(2)<<endlog();
		      
		      
		    }
		    is_arm_actual_received = true;
		  }
		  /// define tool dimensions
		  hand_offset.data[0] = 0;
		  hand_offset.data[1] = 0;
		  hand_offset.data[2] = 0.22;
		  
		  /// yellow knife
		  knife_offset.data[0] = 0;
		  knife_offset.data[1] = 0.035;
		  knife_offset.data[2] = 0.28 + 0.01;
		  /// bread cutter
// 		  knife_offset.data[0] = 0;
// 		  knife_offset.data[1] = 0.025;
// 		  knife_offset.data[2] = 0.28;
		  
		  spoon_offset.data[0] = 0;
		  spoon_offset.data[1] = 0;
		  spoon_offset.data[2] = hand_offset.data[2]+ 0.15;
		  
		  cleaver_offset.data[0] = 0;
		  cleaver_offset.data[1] = 0.09;
		  cleaver_offset.data[2] = 0.35;
		  ///

		  return true;
		}

		void Perception::updateHook()
		{
// 		  //log(Warning)<<"Perception Updatehook!"<<endlog();
// 		  /// receive action type
// 		  int act_typ;
		  /// receive free direction
		  if(!is_free_direction_set&&freeDirectionInport.read(free_direction)==NewData)
		  {
		    free_direction_vector = free_direction.data;
		    //log(Error)<<"Perception:updateHook >> free direction is received= "<<free_direction_vector.at(0)<<endlog();
		    is_free_direction_set = true;
		  }
		  /// receive tool pose
		  geometry_msgs::Pose arm_pose;
		  if(arm_actual_inport.read(arm_pose)==NewData)
		  {
		    float quaternion_norm = sqrt(arm_pose.orientation.x*arm_pose.orientation.x +arm_pose.orientation.y*arm_pose.orientation.y 
		    + arm_pose.orientation.z*arm_pose.orientation.z +arm_pose.orientation.w*arm_pose.orientation.w);
		    
		    update_positions = true;
		    if(std::fabs(quaternion_norm-1.0) > 0.1)
		    {
		      log(Error)<<"Perception:updateHook >> Quaternion norm= "<<quaternion_norm<<endlog();
		      update_positions=false;
		    }
		    if(update_positions)
		    {
		      arm_actual_robot = arm_pose;
		      transform_robot_to_table(arm_actual_robot,arm_actual_table);
  // 		    //log(Warning)<<"arm in robot frame= "<<arm_actual_robot<<endlog();
		      adjust_tool(arm_actual_robot,tool_actual_robot,tool_offset,1);
  // 		    //log(Warning)<<"tool in robot frame= "<<tool_actual_robot<<endlog();
  // 		    //log(Warning)<<"arm in table frame= "<<arm_actual_table<<endlog();
		      adjust_tool(arm_actual_table,tool_actual_table,tool_offset,1);
  // 		    //log(Warning)<<"tool in table frame= "<<tool_actual_table<<endlog();
		      /// show the poses
  // 		    //log(Warning)<<"arm in robot frame= "<<arm_actual_robot<<endlog();
  // 		    //log(Warning)<<"tool in robot frame= "<<tool_actual_robot<<endlog();
		    }
// 		    log(Info)<<"arm in table frame= "<<arm_actual_table<<endlog();
// 		    log(Info)<<"tool offset= "<<tool_offset.data[0]<<" "<<tool_offset.data[1]<<" "<<tool_offset.data[2]<<endlog();
// 		    log(Info)<<"tool in table frame= "<<tool_actual_table<<endlog();
		    actual_tool_pose_outport.write(tool_actual_table);
		    actual_arm_pose_outport.write(arm_actual_table);
// 		    tool_robot_outport.write(tool_actual_robot);
		    if(actual_relations.size()>0)
		    {

		      if(relation_properties.at(2).type!=RELATION_DONTCARE && relation_properties.at(2).sensor==SENSOR_POSITION && actual_relations.at(2)!="T" &&actual_relations.at(0)=="T" && tool_actual_table.position.z <0.1)
		      {
			update_relation(2,"T");
			//log(Warning)<<"perception::updateHook >> Relation 2 is changed to T. Tool touched the background "<<endlog();
			//log(Warning)<<"perception::updateHook >> arm_pose.z = "<<tool_actual_table.position.z<<endlog();
		      }
		      if(relation_properties.at(2).sensor==SENSOR_POSITION && actual_relations.at(2)!="N" &&actual_relations.at(0)=="T" && tool_actual_table.position.z >0.12)
		      {
			update_relation(2,"N");
			//log(Warning)<<"perception::updateHook >> Relation 2 is changed to N. Tool is away from the background "<<endlog();
			//log(Warning)<<"perception::updateHook >> arm_pose.z = "<<tool_actual_table.position.z<<endlog();
		      }
		      /// Relation 4. Detecting touching relation: Rule #1 
		      if(/*action_type!=19&&*/relation_properties.at(4).sensor==SENSOR_POSITION && actual_relations.at(4)!="T" &&
			actual_relations.at(0)=="T" && tool_actual_table.position.z <-0.03)
		      {
			
			log(Warning)<<"perception::updateHook >> Relation 4 is changed to T. Rule #1: tool_actual_table.position.z is less than a threshold  "<<endlog();
			log(Warning)<<"perception::updateHook >> tool_actual_table.position.z= "<<tool_actual_table.position.z<<endlog();
			log(Warning)<<"perception::updateHook >> arm_actual_table.position.z= "<<arm_actual_table.position.z<<endlog();
			update_relation(4,"T");
		      }
		      /// Relation 4. Detecting Not-touching relation: Rule #1
		      if(relation_properties.at(4).sensor==SENSOR_POSITION && actual_relations.at(4)!="N" &&
			actual_relations.at(0)=="T" && tool_actual_table.position.z >0.15)
		      {
			update_relation(4,"N");
			log(Warning)<<"perception::updateHook >> Relation 4 is changed to N. Rule #1: tool_actual_table.position.z is greater than a threshold"<<endlog();
			log(Warning)<<"perception::updateHook >> tool_actual_table.position.z = "<<tool_actual_table.position.z<<endlog();
			log(Warning)<<"perception::updateHook >> arm_actual_table.position.z= "<<arm_actual_table.position.z<<endlog();
		      }

		    }
		  }
		  /// receive wrench valuse
		  geometry_msgs::Wrench wrench;
		  if (wrench_port.read(wrench)==NewData)
		  {
// 		    log(Info)<<"perception::updateHook >> wrench is received from FRI "<<endlog();
// 		    //log(Warning)<<"Perception received NewData from "<<wrench_port.getName()<<endlog();
		    //msr_wrench = wrench;
		    /// get the force in tool frame
		    KDL::Vector force_tool;
		    KDL::Wrench wrench_tool;
		    wrench_tool.force.data[0]= wrench.force.x;
		    wrench_tool.force.data[1]= wrench.force.y;
		    wrench_tool.force.data[2]= wrench.force.z;
		    wrench_tool.torque.data[0]= wrench.torque.x;
		    wrench_tool.torque.data[1]= wrench.torque.y;
		    wrench_tool.torque.data[2]= wrench.torque.z;
		    
		    force_tool[0] = wrench.force.x;
		    force_tool[1] = wrench.force.y;
		    force_tool[2] = wrench.force.z;
// 		    //log(Warning)<<wrench<<endlog();
		    /// convert force from tool frame to robot frame
		    KDL::Frame temp_frame;
		    tf::poseMsgToKDL(arm_actual_robot,temp_frame);
		    KDL::Vector force_robot;
		    KDL::Wrench wrench_robot;
		    KDL::Rotation robot_tool_rotation = temp_frame.M;
		    force_robot = robot_tool_rotation* force_tool;
		    wrench_robot.force = robot_tool_rotation* wrench_tool.force;
		    wrench_robot.torque = robot_tool_rotation* wrench_tool.torque;
		    /// convert force from robot frame to table frame
		    //tf::poseMsgToKDL(arm_actual_table,temp_frame);
// 		    KDL::Rotation table_tool_rotation = table_robot_transform.M;
		    KDL::Vector force_table;
		    KDL::Wrench wrench_table;
		    force_table = table_robot_transform.M* force_robot;
		    wrench_table.force = table_robot_transform.M* wrench_robot.force;
		    wrench_table.torque = table_robot_transform.M* wrench_robot.torque;
		    
// 		    msr_wrench.force.x = force_table.data[0]; 
// 		    msr_wrench.force.y = force_table.data[1];
// 		    msr_wrench.force.z = force_table.data[2];
		    msr_wrench.force.x = wrench_table.force.data[0]; 
		    msr_wrench.force.y = wrench_table.force.data[1];
		    msr_wrench.force.z = wrench_table.force.data[2];
		    msr_wrench.torque.x = wrench_table.torque.data[0]; 
		    msr_wrench.torque.y = wrench_table.torque.data[1];
		    msr_wrench.torque.z = wrench_table.torque.data[2];
		    raw_wrench_port.write(msr_wrench);
// 		    //log(Warning)<<"perception::updateHook >> Raw Wrench= "<<msr_wrench<<endlog();
		    geometry_msgs::Wrench filtered_wrench_new;
		    if(!filtered_wrench_initialized)
		    {
		      filtered_wrench.force.x = msr_wrench.force.x;
		      filtered_wrench.force.y = msr_wrench.force.y;
		      filtered_wrench.force.z = msr_wrench.force.z;
		      filtered_wrench.torque.x = msr_wrench.torque.x;
		      filtered_wrench.torque.y = msr_wrench.torque.y;
		      filtered_wrench.torque.z = msr_wrench.torque.z;
		      filtered_wrench_initialized = true;
		      filtered_wrench_new = filtered_wrench;
		    }
		    else
		    {
		      
		      filtered_wrench_new.force.x = (1-filter_alpha) * filtered_wrench.force.x + filter_alpha*msr_wrench.force.x;
		      filtered_wrench_new.force.y = (1-filter_alpha) * filtered_wrench.force.y + filter_alpha*msr_wrench.force.y;
		      filtered_wrench_new.force.z = (1-filter_alpha) * filtered_wrench.force.z + filter_alpha*msr_wrench.force.z;
		      
		      filtered_wrench_new.torque.x = (1-filter_alpha) * filtered_wrench.torque.x + filter_alpha*msr_wrench.torque.x;
		      filtered_wrench_new.torque.y = (1-filter_alpha) * filtered_wrench.torque.y + filter_alpha*msr_wrench.torque.y;
		      filtered_wrench_new.torque.z = (1-filter_alpha) * filtered_wrench.torque.z + filter_alpha*msr_wrench.torque.z;
		    }
// 		    //log(Warning)<<"perception::updateHook >> Filtered Wrench= "<<filtered_wrench_new<<endlog();
		    filtered_wrench_port.write(filtered_wrench_new);
		    wrench_velocity.force.x = (filtered_wrench_new.force.x - filtered_wrench.force.x) / 0.1;
		    wrench_velocity.force.y = (filtered_wrench_new.force.y - filtered_wrench.force.y) / 0.1;
		    wrench_velocity.force.z = (filtered_wrench_new.force.z - filtered_wrench.force.z) / 0.1;
		    wrench_velocity.torque.x = (filtered_wrench_new.torque.x - filtered_wrench.torque.x) / 0.1;
		    wrench_velocity.torque.y = (filtered_wrench_new.torque.y - filtered_wrench.torque.y) / 0.1;
		    wrench_velocity.torque.z = (filtered_wrench_new.torque.z - filtered_wrench.torque.z) / 0.1;
		    wrench_velocity_port.write(wrench_velocity);
		    
		    // contact detection for force x,y,z axes for both negative or positive directions
		    // for cutting, the threshold is lower, since the force between knife and object is small
		    if(action_type==10)
		      contact_threshold_positive = 1.4;
		    else if(action_type==14 || action_type==16 || action_type==17|| action_type==18  ||action_type==3)
		    {
		      contact_threshold_positive = 5;
		    }
		    else
		      contact_threshold_positive = 4;
		    contact_threshold_negative = -1*contact_threshold_positive;
		    std::vector<short> int_vector(3,0);
		    contact_detected.data = int_vector;
		    if(wrench_velocity.force.x < contact_threshold_negative) contact_detected.data.at(0) = 1;
		    if(wrench_velocity.force.y < contact_threshold_negative) contact_detected.data.at(1) = 1;
		    if(wrench_velocity.force.z < contact_threshold_negative) contact_detected.data.at(2) = 1;
		    if(wrench_velocity.force.x > contact_threshold_positive) contact_detected.data.at(0) = -1;
		    if(wrench_velocity.force.y > contact_threshold_positive) contact_detected.data.at(1) = -1;
		    if(wrench_velocity.force.z > contact_threshold_positive) contact_detected.data.at(2) = -1;
		    contact_detected_outport.write(contact_detected);
		    // visualize contact detection in console
// 		    //log(Warning)<<"perception::updateHook >> Contact Detected";
// 		    for(int i=0; i<(int) contact_detected.data.size();i++)
// 		    {
// 		      if(contact_detected.data.at(i)==1)
// 			//log(Warning)<<"###############";
// 		      else if(contact_detected.data.at(i)==-1)
// 			//log(Warning)<<"...............";
// 		      else
// 			//log(Warning)<<"               ";
// 		      
// 		    }
// 		    //log(Warning)<<endlog();
// 		    //log(Warning)<<wrench_velocity<<endlog();
// 		    log(Info)<<"perception::updateHook >> filtered wrench is sent "<<endlog();
		    filtered_wrench = filtered_wrench_new;
		    if(actual_relations.size()>0 ) 
		    {
		      /// relation 0 (tool,obj1) with force ---> for pushing
		      /// if change of force in XY direction is more than a threshold and arm is moving, change the relation 0 to touching
// 		      float force_velocity_xy = sqrt(wrench_velocity.force.x*wrench_velocity.force.x + wrench_velocity.force.y*wrench_velocity.force.y);
		      KDL::Vector force_velocity;
		      force_velocity.data[0] = wrench_velocity.force.x;
		      force_velocity.data[1] = wrench_velocity.force.y;
		      force_velocity.data[2] = wrench_velocity.force.z;
		      double dot_product = dot(movement_direction,force_velocity);
		      double opposing_force = dot_product;
		      opposing_force_outport.write(opposing_force);
		      //float force_velocity_xy = 0; 
		      /// this is only for pushing, which is not part of the library.
		      if (action_type==11 && relation_properties.at(0).sensor==SENSOR_FORCE && opposing_force>1.2 && actual_relations.at(0)=="N" 
			&& arm_status==ARM_MOVING && distance_xy(tool_actual_table,obj1_table)<0.10)
		      {
			//log(Warning)<<"perception::updateHook >> Tool touched Object1, changing relation 0 to T "<<endlog();
			//log(Warning)<<"perception::updateHook >> distance XY is= "<<distance_xy(tool_actual_table,obj1_table)<<endlog();
			//log(Warning)<<"perception::updateHook >> opposing_force is= "<<opposing_force<<endlog();
			//log(Warning)<<"perception::updateHook >> wrench_velocity is= "<<wrench_velocity<<endlog();
			//log(Warning)<<"perception::updateHook >> dot_product is= "<<dot_product<<endlog();
			//log(Warning)<<"perception::updateHook >> force velocity norm is= "<<force_velocity.Norm()<<endlog();
			
			update_relation(0,"T");
		      }
		      /// relation 0 (tool,obj1) with force
// 		      contact_threshold_negative = 3;
		      /// if change of force in Z direction is more than a threshold and arm is moving, change the relation 0 to touching
		      if (relation_properties.at(0).sensor==SENSOR_FORCE && wrench_velocity.force.z<contact_threshold_negative && actual_relations.at(0)!="T" 
			&& arm_status==ARM_MOVING && tool_actual_table.position.z>(obj1_table.position.z-0.01)&& distance_xy(tool_actual_table,obj1_table)<0.10)
		      {
			log(Warning)<<"perception::updateHook >> Tool touched Object1, changing relation 0 to T "<<endlog();
			log(Warning)<<"perception::updateHook >> tool Z is= "<<tool_actual_table.position.z<<endlog();
			log(Warning)<<"perception::updateHook >> F_vel Z is= "<<wrench_velocity.force.z<<endlog();
			update_relation(0,"T");
		      }
		      /// if change of force in Z direction is more than a threshold and arm is moving, change the relation 0 to unknown
		      if (relation_properties.at(0).sensor==SENSOR_FORCE && wrench_velocity.force.z>contact_threshold_positive && actual_relations.at(0)=="T" 
			&& arm_status==ARM_MOVING &&  distance_xy(tool_actual_table,obj1_table)>0.03)
		      {
			log(Warning)<<"perception::updateHook >> Tool untouched Object1, changing relation 0 to N "<<endlog();
			log(Warning)<<"perception::updateHook >> tool Z is= "<<tool_actual_table.position.z<<endlog();
			log(Warning)<<"perception::updateHook >> F_vel Z is= "<<wrench_velocity.force.z<<endlog();
			update_relation(0,"A");
		      }
		      /// Relation 4. Detecting touching relation: Rule #2
		      // if change of force in Z direction is more than a threshold and arm is moving, change the relation 4 to touching
		      if (relation_properties.at(4).sensor==SENSOR_FORCE  && actual_relations.at(4)=="N" && actual_relations.at(0)=="T"
			  && wrench_velocity.force.z<contact_threshold_negative && arm_status==ARM_MOVING && tool_actual_table.position.z<0.10)
		      {
			log(Warning)<<"perception::updateHook >> Relation 4 is changed to T. Rule #2: Contact is detected between obj1 and background  "<<endlog();
			log(Warning)<<"perception::updateHook >> object1 touched background, changing relation 4 to T "<<endlog();
			log(Warning)<<"perception::updateHook >> tool_actual_table.position.z= "<<tool_actual_table.position.z<<endlog();
			log(Warning)<<"perception::updateHook >> wrench_velocity.force.z= "<<wrench_velocity.force.z<<endlog();
			update_relation(4,"T");
		      }
		      /// Relation 3. Detecting touching relation: Rule #1
		      // relation 3 (obj1,obj2) with force
		      if (relation_properties.at(3).sensor==SENSOR_FORCE && actual_relations.at(3)=="N" && actual_relations.at(0)=="T" &&
			  (wrench_velocity.force.z<contact_threshold_negative/*||wrench_velocity.force.z>contact_threshold_positive*/) &&
			  arm_status==ARM_MOVING /*&& tool_actual_table.position.z>obj2_table.position.z*/ /*&& distance_xy(tool_actual_table,obj2_table)<0.10 */
// 			  && std::abs(tool_actual_table.position.z-obj2_table.position.z)<0.06
		      )
		      {
			if(distance_xy(tool_actual_table,obj2_table)>0.10 /*|| std::abs(tool_actual_table.position.z-obj2_table.position.z)>0.06
			  || tool_actual_table.position.z<obj2_table.position.z*/)
			{
			  log(Warning)<<"perception::updateHook >> Force is detected between obj1 and obj2 but position is incorrect"<<endlog();
			  log(Warning)<<"perception::updateHook >> distance_xy="<<distance_xy(tool_actual_table,obj2_table)<<endlog();
			  log(Warning)<<"perception::updateHook >> z distance="<<std::abs(tool_actual_table.position.z-obj2_table.position.z)<<endlog();
			  log(Warning)<<"perception::updateHook >> tool_actual_table="<<tool_actual_table<<endlog();
			  log(Warning)<<"perception::updateHook >> obj2_table="<<obj2_table<<endlog();
			}
			
			else
			{
			  log(Warning)<<"perception::updateHook >> Relation 3 is changed to T. Rule #1: Contact is detected between obj1 and obj2  "<<endlog();
			  log(Warning)<<"perception::updateHook >> object1 touched object2, changing relation 3 to T "<<endlog();
			  log(Warning)<<"perception::updateHook >> distance_xy is= "<<distance_xy(tool_actual_table,obj2_table)<<endlog();
			  log(Warning)<<"perception::updateHook >> tool_actual_table.position.z is= "<<tool_actual_table.position.z<<endlog();
			  log(Warning)<<"perception::updateHook >> arm_actual_table.position.z= "<<arm_actual_table.position.z<<endlog();
			  log(Warning)<<"perception::updateHook >> obj2_z is= "<<obj2_table.position.z<<endlog();
			  log(Warning)<<"perception::updateHook >> F_vel Z is= "<<wrench_velocity.force.z<<endlog();
			  update_relation(3,"T");
			}
		      }
		    }

		  }
		  /// receive the relations properties
		  if(relation_properties_inport.read(relation_properties)==NewData)
		  {
		    relation_properties_is_set = true;
		    std::vector<short> rel_type_vec,rel_sensor_vec;
		    for(int i=0; i<relation_properties.size();i++)
		    {
		      rel_type_vec.push_back((short)relation_properties.at(i).type);
		      rel_sensor_vec.push_back((short)relation_properties.at(i).sensor);
		    }
		    std_msgs::Int16MultiArray rel_type,rel_sensor;
		    rel_type.data = rel_type_vec;
		    rel_sensor.data = rel_sensor_vec;
		    relation_type_int_msgs_outport.write(rel_type);
		    relation_sensor_int_msgs_outport.write(rel_sensor);
		  }
		  /// receive tool_type
		  ROBOT_TOOL_TYPE tooltype;
		  if(tool_type_inport.read(tooltype)==NewData)
		  {
// 		      tool_type_is_set = (tooltype==tool_type);
// 		      if (tooltype!=tool_type)
// 			tool_type_is_set=false;
// 		      else
// 			tool_type_is_set=;
		    if(tooltype!=tool_type)
		    {
		      tool_type = tooltype;
		      //log(Warning)<<"perception::updateHook >> Tool type is recevied= "<<tool_type<<endlog();
		      tool_type_is_set = true;
		      set_tool_offset(tooltype);
		      
		    }
		  }
		  ///receive init relations
		  std::string rel1_init,rel2_init,rel3_init;
		  if(init_relation1_inport.read(rel1_init)==NewData && !is_relation_1_init_received)
		  {
		    //log(Warning)<<"perception::updateHook >> init relations 1 is received from rosinterface= "<<rel1_init<<endlog();
		    relation_1_init = rel1_init;
		    is_relation_1_init_received = true;
		  }
		  if(init_relation2_inport.read(rel2_init)==NewData && !is_relation_2_init_received)
		  {
		    //log(Warning)<<"perception::updateHook >> init relations 2 is received from rosinterface= "<<rel2_init<<endlog();
		    relation_2_init = rel2_init;
		    is_relation_2_init_received = true;
		  }
		  if(init_relation3_inport.read(rel3_init)==NewData && !is_relation_3_init_received)
		  {
		    
		    //log(Warning)<<"perception::updateHook >> init relations 3 is received from rosinterface= "<<rel3_init<<endlog();
		    relation_3_init = rel3_init;
		    is_relation_3_init_received = true;
		  }
		  /// receive pose values
		  geometry_msgs::Pose pose1;
		  if(obj1_pose_inport.read(pose1)==NewData)
		  {
		    //log(Error)<<"perception::updateHook >> received NewData from "<<obj1_pose_inport.getName()<<endlog();
		    obj1_camera = pose1;
		    obj1_table = pose1;
		    obj1_pose_is_set = true;
		    obj1_angle_is_set = true;
		    /// threshod for elongated objects vs round objects
		    if (obj1_table.orientation.w > 2.5&& action_type!=8 /*&&false*/)
		    {
		      is_obj1_round = false;
		      is_obj1_round_msgs.data = is_obj1_round;
		      is_obj1_round_msgs_outport.write(is_obj1_round_msgs);
		      //log(Warning)<<"perception::updateHook >> obj1 is elongated. The ratio is= "<<obj1_table.orientation.w<<endlog();
		      KDL::Vector obj1_pc_camera,obj1_pc_table;
		      obj1_pc_table.data[0] = obj1_table.orientation.x;
		      obj1_pc_table.data[1] = obj1_table.orientation.y;
		      obj1_pc_table.data[2] = obj1_table.orientation.z;
		      
		      //log(Warning)<<"perception::updateHook >> obj1 first principle component in table frame is= "<<obj1_pc_table.data[0]<<" "
// 			<<obj1_pc_table.data[1]<<" "<<obj1_pc_table.data[2]<<endlog();
		      obj1_angle = 180*atan2(obj1_pc_table.data[1],obj1_pc_table.data[0])/M_PI;
		      //log(Warning)<<"perception::updateHook >> obj1 angle is= "<<obj1_angle<<endlog();
		    }
		    else
		    {
		      is_obj1_round = true;
		      is_obj1_round_msgs.data = is_obj1_round;
		      is_obj1_round_msgs_outport.write(is_obj1_round_msgs);
		      //log(Warning)<<"perception::updateHook >> obj1 is round (or cosidered round for this action). The ratio is= "<<obj1_table.orientation.w<<endlog();
		    }
		  }
		  float angle1;
		  if(obj1_angle_inport.read(angle1)==NewData)
		  {
		    //obj1_angle = angle1;
// 		    obj1_angle_is_set = true;
		  }
		  geometry_msgs::Pose pose2;
		  if(obj2_pose_inport.read(pose2)==NewData)
		  {
		    //log(Warning)<<"Perception::updateHook >> received NewData from "<<obj2_pose_inport.getName()<<endlog();
		    obj2_camera = pose2;
		    obj2_table = pose2;
		    obj2_pose_is_set = true;
		    obj2_angle_is_set = true;
		    if (obj2_table.orientation.w > 1.8 && action_type!=8/*&&false*/)
		    {
		      is_obj2_round = false;
		      is_obj2_round_msgs.data = is_obj2_round;
		      is_obj2_round_msgs_outport.write(is_obj2_round_msgs);
		      //log(Warning)<<"perception::updateHook >> obj2 is elongated. The ratio is= "<<obj2_table.orientation.w<<endlog();
		      KDL::Vector obj2_pc_camera,obj2_pc_table;
		      obj2_pc_table.data[0] = obj2_table.orientation.x;
		      obj2_pc_table.data[1] = obj2_table.orientation.y;
		      obj2_pc_table.data[2] = obj2_table.orientation.z;
		      //log(Warning)<<"perception::updateHook >> obj2 first component in table frame is= "<<obj2_pc_table.data[0]<<" "
// 			<<obj2_pc_table.data[1]<<" "<<obj2_pc_table.data[2]<<endlog();
		      obj2_angle = 180*atan2(obj2_pc_table.data[1],obj2_pc_table.data[0])/M_PI;
		      //log(Warning)<<"perception::updateHook >> obj2 angle is= "<<obj2_angle<<endlog();
		    }
		    else
		    {
		      is_obj2_round = true;
		      is_obj2_round_msgs.data = is_obj2_round;
		      is_obj2_round_msgs_outport.write(is_obj2_round_msgs);
		      //log(Warning)<<"perception::updateHook >> obj2 is round. The ratio is= "<<obj2_table.orientation.w<<endlog();
		    }
		  }
		  geometry_msgs::Pose goal_temp;
		  if(goal_pose_inport.read(goal_temp)==NewData)
		  {
		    //log(Warning)<<"Perception::updateHook >> received NewData from "<<goal_pose_inport.getName()<<endlog();
		    
		    goal_table = goal_temp;
		    
		  }
		  /// read the last three relations from vision, if there is an input on port
		  std::vector<std::string> vision_rel; 
		  if(relations_vision_inport.read(vision_rel)==NewData)
		  {
		    //log(Warning)<<"perception::updateHook >> received NewData from "<<relations_vision_inport.getName()<<endlog();
// 		    //log(Warning)<<"perception::updateHook >> received vistion relations are= "<<vision_rel<<endlog();
		    /// relations 3,4,5 are set from vision
		    for(int i=3;i<6;i++)
		    {
		      if (i<(int)actual_relations.size())
		      {
			//actual_relations.at(i)= vision_rel.at(i);
			update_relation(i,vision_rel.at(i));
		      }
		      else
			log(Error)<<"Perception::updateHook: actual_relations index is out of- range, relation is not updated from vision"<<endlog();
		    }
		  }
		  /// by enum types
		  if(hand_status_inport2.read(hand_status)==NewData)
		  {
		    //log(Warning)<<"perception::updateHook >> received NewData from "<<hand_status_inport2.getName()<<endlog();
		    //log(Warning)<<"perception::updateHook >> received hand status is= "<<hand_status<<endlog();
		    hand_status_int = static_cast <int> (hand_status);
		    if(hand_status==HAND_GRASPED)
		    {
		      if(arm_status==ARM_AT_OBJ1 && relation_properties.at(0).sensor==SENSOR_TACTILE)
		      {
			//log(Warning)<<"perception::updateHook >> Relation 0 is changed to T because hand grasped obj1"<<endlog();
			update_relation(0,"T");
		      }
		      if(arm_status==ARM_AT_OBJ2 && relation_properties.at(1).sensor==SENSOR_TACTILE)
		      {
			//log(Warning)<<"perception::updateHook >> Relation 1 is changed to T because hand grasped obj2"<<endlog();
			update_relation(1,"T");
		      }
		    }
		    else if(hand_status!=HAND_UNDEFINED)
		    {
		      update_relation(0,"N");
		      //update_relation(1,"N");
		    }
		  }
		  /// read changes in arm status
		  std::string arm_event;
		  if(arm_status_inport.read(arm_event)==NewData)
		  {
		    //log(Warning)<<"perception::updateHook >> received NewData from "<<arm_status_inport.getName()<<endlog();
		    //log(Warning)<<"perception::updateHook >> received arm event is= "<<arm_event<<endlog();
		    if(arm_event=="e_cartesianGeneratorPosDMP_move_started")
		    {
		      arm_status_string = "arm_moving";
		      arm_status = ARM_MOVING;
		    }
		    else if(arm_event=="e_cartesianGeneratorPosDMP_move_finished")
		    {
		      /// if robot stops moving, we check its desired goal, and assume that it has reached this point
		      /// however, this is not the correct way, we should match the current position of robot with the desired point,
		      /// and change the arm_status only if the robot is actually there (how to determine that?), otherwise, the arm_status is undefined
		      /// todo: compare the current arm pose to the desired pose (or all desired poses)
		      ///       to see if the robot arm is actually at the desired pose
		      calculate_current_arm_goal();
		      set_arm_status();
		      /// calculating the relation 3,4,5, this is temporary replacement for vision
		      /// relation 4 : obj1 and background
		      //log(Warning)<<"perception::updateHook >> arm_status received: "<<arm_status<<endlog();
		      /// going to Home has different effect in Cutting action than other actions
		      if(relation_properties.size()>0)
		      {
			if(relation_properties.at(0).sensor!=SENSOR_TACTILE) // cutting and stirring
			{
  // 			if( actual_relations.at(0)=="T" && arm_status==ARM_AT_HOME)
  // 			  update_relation(0,"N");
			//}
			//else if(action_type==9) // stir
			//{
			  if( actual_relations.at(0)=="T" && arm_status==ARM_AT_HOME)
			  {
			    update_relation(0,"N");
			    //log(Warning)<<"perception::updateHook >> Relation 0 is changed to N because arm is at home"<<endlog();
			    ////log(Warning)<<"perception::updateHook >> arm is at home, update relation 0 to N "<<endlog();
			  }
			  if( relation_properties.at(0).sensor==SENSOR_VISION && actual_relations.at(0)=="N" && arm_status==ARM_AT_OBJ1)
			  {
			    update_relation(0,"T");
			    //log(Warning)<<"perception::updateHook >> Relation 0 is changed to T because arm is at obj1"<<endlog();
  // 			  //log(Warning)<<"perception::updateHook >> arm is at obj1, update relation 0 to T "<<endlog();
			  }
			}
			else //relation_properties.at(0).sensor==SENSOR_TACTILE 
			{
			  
			  /// Relation 4. Detecting touching relation: Rule #3 
			  if( relation_properties.at(4).sensor==SENSOR_POSITION && actual_relations.at(4)!="T" && actual_relations.at(0)=="T"&&
			    (arm_status==ARM_AT_GOAL||arm_status == ARM_AT_BACKGROUND) ) 
			  {
			    
			    update_relation(4,"T");
			    log(Warning)<<"perception::updateHook >> Relation 4 is changed to T. Rule #3: Arm is at goal or background"<<endlog();
			  }
			  /// Relation 4. Detecting not-touching relation: Rule #2 
			  if(actual_relations.at(4)!="N"  && actual_relations.at(0)=="T" &&
			  (arm_status == ARM_AT_HOME2||arm_status==ARM_AT_HOME||arm_status==ARM_AT_ERROR || arm_status==ARM_AT_GOAL2) )
			  {
			    update_relation(4,"N");
			    //log(Warning)<<"perception::updateHook >> Relation 4 is changed to N. Rule #2: Obj1 reached Home,Home2,Error or Goal2 points"<<endlog();
			  }

			
			
			  // TODO: need to add more conditions: position of tool and obj2
			  /// Relation 3. Detecting touching relation: Rule #2 
			  if(actual_relations.at(3)!="T" && relation_properties.at(3).sensor == SENSOR_POSITION && arm_status == ARM_AT_BACKGROUND
			    && actual_relations.at(0)=="T")
			  {
			    update_relation(3,"T");
			    //log(Warning)<<"perception::updateHook >> Relation 3 is changed to T. Rule #2: Tool is holding obj1 and it is at the background (condition should be corrected)"<<endlog();
			  }
			  /// Relation 3. Detecting Not-touching relation: Rule #1 
			  if(actual_relations.at(3)!="N" && actual_relations.at(0)=="T" &&
			    (arm_status == ARM_AT_HOME2 ||arm_status==ARM_AT_HOME||arm_status==ARM_AT_ERROR|| arm_status==ARM_AT_GOAL2) )
			  {
			    update_relation(3,"N");
			    //log(Warning)<<"perception::updateHook >> Relation 3 is changed to N. Rule #1: Arm is at Home, Home2, Goal2 or Error"<<endlog();
			  }
			  /// e.g. pick and place, put on top
  // 			if(actual_relations.at(4)!="N" && actual_relations.at(0)=="T" && (arm_status==ARM_AT_HOME||arm_status==ARM_AT_ERROR))
  // 			{
  // 			  update_relation(4,"N");
  // 			  //log(Warning)<<"perception::updateHook >> Relation 4 is changed to N because arm is at home or error"<<endlog();
  // 			}
			  /// e.g. take down
// 			  if(/*action_type!=8 &&*/actual_relations.at(3)!="N" && actual_relations.at(0)=="T" && (arm_status==ARM_AT_HOME2||arm_status==ARM_AT_HOME||arm_status==ARM_AT_ERROR))
// 			  {
// 			    update_relation(3,"N");
// 			    //log(Warning)<<"perception::updateHook >> Relation 3 is changed to N because arm is at home or error"<<endlog();
// 			  }
			}
		      
			/// put-in (hide_in)
			if(action_type==4&& actual_relations.at(3)=="N" && actual_relations.at(0)=="T" && arm_status==ARM_AT_OBJ2)
			{
  // 			if(action_type==4) /// hide_in
			  update_relation(3,"A");
			  //log(Warning)<<"perception::updateHook >> Relation 3 is changed to A because arm is at obj2"<<endlog();
  // 			else
  // 			  log(Error)<< " Move primitive finished but relations are not changed! "<<endlog();
  // 			update_relation(3,"T");
			}
			if( action_type==4 && actual_relations.at(4)=="N" && actual_relations.at(0)=="T" && arm_status==ARM_AT_OBJ2 && actual_relations.at(3)=="A")
			{
			    update_relation(4,"A");
			    //log(Warning)<<"perception::updateHook >> Relation 4 is changed to A because arm is at obj2"<<endlog();
			}
			/// Relation 3. Detecting touching relation: Rule #3
			if( action_type==9 && actual_relations.at(3)!="T" && actual_relations.at(0)=="T" && arm_status==ARM_AT_OBJ2)
			{
			    update_relation(3,"T");
			    //log(Warning)<<"perception::updateHook >> Relation 3 is changed to T because arm is at obj2 (stirring)"<<endlog();
			}
		      
		      ///
// 		      if(actual_relations.at(0)=="T" && arm_status==ARM_AT_GOAL2 && actual_relations.at(3)!="N")
// 		      {
// 			////log(Warning)<<"perception::updateHook >> inside if "<<arm_status<<endlog();
// 			//update_relation(4,"T");
// // 			if(actual_relations.at(3)=="T" && action_type==8) /// pour
// // 			{
// 			  update_relation(3,"N");
// 			  //log(Warning)<<"perception::updateHook >> Relation 3 is changed to N because arm is at goal2"<<endlog();
// // 			  //log(Warning)<<"perception::updateHook >> pouring is done! Relation 3 is N"<<endlog();
// // 			}
// 			
// 		      }
		      /// e.g. unload
// 		      if(actual_relations.at(0)=="T" && arm_status==ARM_AT_GOAL2 && actual_relations.at(4)=="T")
// 		      {
// 			////log(Warning)<<"perception::updateHook >> inside if "<<arm_status<<endlog();
// 			//update_relation(4,"T");
// // 			if(actual_relations.at(3)=="T" && action_type==8) /// pour
// // 			{
// 			  update_relation(4,"N");
// 			  //log(Warning)<<"perception::updateHook >> Relation 4 is changed to N because arm is at goal2"<<endlog();
// // 			  //log(Warning)<<"perception::updateHook >> pouring is done! Relation 3 is N"<<endlog();
// // 			}
// 			
// 		      }
			/// for actions which relation 0 is sensed with force not with tactile.
			if(actual_relations.at(0)=="T" && arm_status==ARM_AT_HOME2 && relation_properties.at(0).sensor==SENSOR_FORCE)
			{
			  ////log(Warning)<<"perception::updateHook >> inside if "<<arm_status<<endlog();
			  //update_relation(4,"T");
  // 			if(actual_relations.at(3)=="T" && action_type==8) /// pour
  // 			{
			    update_relation(0,"N");
			    //log(Warning)<<"perception::updateHook >> Relation 0 is changed to N because arm is at home2"<<endlog();
  // 			  //log(Warning)<<"perception::updateHook >> pouring is done! Relation 3 is N"<<endlog();
  // 			}
			  
			}

		      }
		      
		    }
		  }
		}

		void Perception::stopHook()
		{
		  
		}

		void Perception::cleanupHook()
		{
		  
		}
		bool Perception::set_pose()
		{
		  //log(Error)<<"perception::set_pose >> starts! "<<endlog();
		  if(!obj1_pose_is_set || !obj2_pose_is_set || !action_type_is_set || !obj1_angle_is_set)
		  {
		    obj_set_outport.write(false);
		    log(Error)<<"perception::set_pose >> failure because of unset flags: "<<
		      obj1_pose_is_set<<" "<<obj2_pose_is_set<<" "<<action_type_is_set<<" "<<obj1_angle_is_set<<endlog();
		    
		    return false;
		  }
		  
		  /// initialize the orientation of objects. these default values will be modified if needed. except for cutting!
		  if(action_type!= 10)
		  {
		    obj1_table.orientation.x = -0.70710678118;
		    obj1_table.orientation.y = 0.70710678118;
		    obj1_table.orientation.z = 0;
		    obj1_table.orientation.w = 0;
		    obj2_table.orientation = obj1_table.orientation;
		  }
		  
		  float angle_change_B = -30* free_direction_vector.at(1);
		  float angle_change_C = 30* free_direction_vector.at(0);
		  //set_orientation(obj1_table,0,0+angle_change_B,180+angle_change_C);
		  //obj1_table.position.x -=  0.06;
		  //obj1_table.position.y +=  0.04;
		  //obj1_table.position.z -=  0.02;
		  
		  obj2_table.position.y -=  0.04;
// 		  obj1_table.orientation.x = 0.496104;
// 		  obj1_table.orientation.y = 0.868201;
// 		  obj1_table.orientation.z = -0.00332606;
// 		  obj1_table.orientation.w = 0.00982785;
 
		  /// converting from table frame to robot frame
		  //log(Warning)<<"perception::set_pose >> obj1 pose in table frame= "<< obj1_table<<endlog();
		  //log(Warning)<<"perception::set_pose >> obj2 pose in table frame= "<< obj2_table<<endlog();
// 		  obj1_msgs_robot = obj1_table;
// 		  obj2_msgs_robot = obj2_table;
// 		  float x_robot = cos(M_PI/6.0)*obj1_table.position.x - sin(M_PI/6.0)*obj1_table.position.y;
// 		  float y_robot = sin(M_PI/6.0)*obj1_table.position.x + cos(M_PI/6.0)*obj1_table.position.y;
// 		  obj1_msgs_robot.position.x = x_robot;
// 		  obj1_msgs_robot.position.y = y_robot;
		  
// 		  x_robot = cos(M_PI/6.0)*obj2_table.position.x - sin(M_PI/6.0)*obj2_table.position.y;
// 		  y_robot = sin(M_PI/6.0)*obj2_table.position.x + cos(M_PI/6.0)*obj2_table.position.y;
// 		  obj2_msgs_robot.position.x = x_robot;
// 		  obj2_msgs_robot.position.y = y_robot;
// 		  //log(Warning)<<"perception::set_pose >> obj1 pose in robot frame= "<< obj1_msgs_robot<<endlog();
// 		  //log(Warning)<<"perception::set_pose >> obj2 pose in robot frame= "<< obj2_msgs_robot<<endlog();
		  /// prevent the TCP to move to a point too low (and hit the table)
		  float minimum_height = 0.02;
		  if(obj1_table.position.z <minimum_height)
		  {
		    //log(Error)<<"perception::set_pose >> obj1 is too low, using minimun allowed height "<<endlog();
		    obj1_table.position.z = minimum_height;
		  }
		  /// if object1 is elongated, adjust the orientation of obj1_table accordingly
		  if(!is_obj1_round)
		  {
		    /// angle must be between 0 and 180 degrees.
		    //log(Warning)<<"perception::set_pose >> for elongated objects, the orientation of obj1_table is modified to match its main axis"<<endlog();
		    //log(Warning)<<"perception::set_pose >> obj1_angle= "<<obj1_angle<<endlog();
		    if (obj1_angle>0)
		    {
		      //log(Warning)<<"perception::set_pose >> obj1_angle is positive, adding -180 degrees to shift it to the interval [-180 0]"<<endlog();
		      obj1_angle -= 180;
		      //log(Warning)<<"perception::set_pose >> Now, obj1_angle= "<<obj1_angle<<endlog();
		    }
		    set_orientation(obj1_table,obj1_angle,0+angle_change_B,-180+angle_change_C);
		    //log(Warning)<<"perception::set_pose >> modified obj1_table is="<<obj1_table<<endlog();
		    //set_orientation(goal_table,30,0,-180);
		  }
		  /// if object2 is elongated, adjust the orientation of obj2_table accordingly
		  if(!is_obj2_round)
		  {
		    /// angle must be between 0 and 180 degrees.
		    //log(Warning)<<"perception::set_pose >> for elongated objects, the orientation of obj2_table is modified to match its main axis"<<endlog();
		    //log(Warning)<<"perception::set_pose >> obj2_angle= "<<obj2_angle<<endlog();
		    if (obj2_angle>0)
		    {
		      //log(Warning)<<"perception::set_pose >> obj2_angle is positive, adding -180 degrees to shift it to the interval [-180 0]"<<endlog();
		      obj2_angle -= 180;
		      //log(Warning)<<"perception::set_pose >> Now, obj2_angle= "<<obj2_angle<<endlog();
		    }
		    set_orientation(obj2_table,obj2_angle,0+angle_change_B,-180+angle_change_C);
		    //log(Warning)<<"perception::set_pose >> modified obj2_table is="<<obj2_table<<endlog();
		    //set_orientation(goal_table,30,0,-180);
		  }
		  /// for some actions, even for elongated objects, there is no need to use the angle,
		  if(action_type==13 || action_type==14 || action_type==16 || action_type==17 )
		  {
// 		    obj1_table.orientation.x = -0.70710678118;
// 		    obj1_table.orientation.y = 0.70710678118;
// 		    obj1_table.orientation.z = 0;
// 		    obj1_table.orientation.w = 0;
		    
		    obj1_table.orientation.x = -1;
		    obj1_table.orientation.y = 0;
		    obj1_table.orientation.z = 0;
		    obj1_table.orientation.w = 0;
		    
		    obj2_table.orientation = obj1_table.orientation;
		  }
		  /// correct the positions coming from vision, error due to miscalibration and incomplete object perception
// 		  obj1_table.position.x -= 0.03;
// 		  obj1_table.position.y -= 0.02;
		  /// Use fixed positions for the desired demo
		  bool fixed_poses_salad_demo=false;
		  bool short_demo= false;
		  bool fixed_simple_demo= true;
		  //log(Warning)<<"perception::set_pose >> Salad scenraio: action selected= "<<action_type<<endlog();
		  /// fixed demo positions
		  geometry_msgs::Pose p1,p2,p3;
// 		  bowl_pose.position.x = -0.2;
// 		  bowl_pose.position.y = 0.5;
		  p1.position.x = -0.46;
		  p1.position.y = 0.54;
		  p1.position.z = 0.035;
		  
		  p2.position.x = -0.23;
		  p2.position.y = 0.54;
		  p2.position.z = 0.04;
		  
		  p3.position.x = -0.35;
		  p3.position.y = 0.55;
		  p3.position.z = 0.015;
		  /// bowl position
		  geometry_msgs::Pose bowl_pose;
// 		  bowl_pose.position.x = -0.2;
// 		  bowl_pose.position.y = 0.5;
		  bowl_pose.position.x = -0.29;
		  bowl_pose.position.y = 0.42;
		  bowl_pose.position.z = 0.28;
		  if(fixed_simple_demo)
		  {
		    switch(action_type)
		    {
		      /// pick and place
		      case 1: 
		      {
			obj1_table.position.x = p2.position.x;
			obj1_table.position.y = p2.position.y;
			obj1_table.position.z = p2.position.z;
// 			obj1_angle = -90;
// 			set_orientation(obj1_table,obj1_angle,0,-180);
			is_obj1_round = true;
			
// 			obj2_table = obj1_table;
// 			obj2_table.position.x = p2.position.x;
// 			obj2_table.position.y = p2.position.y;
// 			obj2_table.position.z = p2.position.z- 0.01;
			break;
		      }
		      /// put on top
		      case 2: 
		      {
			obj1_table.position.x = p1.position.x;
			obj1_table.position.y = p1.position.y;
			obj1_table.position.z = p1.position.z;
// 			obj1_angle = -90;
// 			set_orientation(obj1_table,obj1_angle,0,-180);
			is_obj1_round = true;
			
			obj2_table = obj1_table;
			obj2_table.position.x = p2.position.x;
			obj2_table.position.y = p2.position.y;
			obj2_table.position.z = p2.position.z;
			break;
		      }
		      /// take down
		      case 3: 
		      {
			obj1_table.position.x = p3.position.x;
			obj1_table.position.y = p3.position.y;
			obj1_table.position.z = p3.position.z + 0.02;
// 			obj1_angle = -90;
// 			set_orientation(obj1_table,obj1_angle,0,-180);
			is_obj1_round = true;
			
			obj2_table = obj1_table;
// 			obj2_table.position.x = -0.145- 0.05 -0.01;
// 			obj2_table.position.y = 0.6-0.03 - 0.05;
			obj2_table.position.z = 0.025;
			break;
		      }
		      /// push by holding
		      case 14: 
		      {
			obj1_table.position.x = p1.position.x;
			obj1_table.position.y = p1.position.y;
			obj1_table.position.z = p1.position.z;
			is_obj1_round = true;
		      }
		      default:
		      {
			//log(Warning)<<"perception::set_pose >> Salad scenraio: Unknown action selected! "<<endlog();
		      }
		    } // end switch
		  } // end if (fixed_simple_demo)
		  if(fixed_poses_salad_demo)
		  {
		    switch(action_type)
		    {
		      /// put on top
		      case 2: 
		      {
			obj1_table.position.x = -0.4- 0.02;
			obj1_table.position.y = 0.74-0.07/*-0.28*/;
			obj1_table.position.z = 0.025- 0.01;
			obj1_angle = -90;
			set_orientation(obj1_table,obj1_angle,0,-180);
			is_obj1_round = false;
			
			obj2_table = obj1_table;
			obj2_table.position.x = -0.145- 0.05 -0.01;
			obj2_table.position.y = 0.6-0.03 - 0.05;
			obj2_table.position.z = 0.025;
			break;
		      }
		      /// cut
		      case 10: 
		      {
			/// mikkel grasp planning
// 			obj1_table.position.x -= 0.03;
// 			obj1_table.position.y -= 0.01;
// 			// for safety
// 			if(obj1_table.position.z < 0.18)
// 			  obj1_table.position.z = 0.18;
// 			//log(Warning)<<"perception::set_pose >> adjust knife offset. obj1.z= "<< obj1_table.position.z<<endlog();
// 			//log(Warning)<<"perception::set_pose >> knife_offset.z before= "<<knife_offset.data[2]<<endlog();
// 			knife_offset.data[2] += obj1_table.position.z - 0.19;
// 			//log(Warning)<<"perception::set_pose >> knife_offset.z after= "<<knife_offset.data[2]<<endlog();
// 			set_tool_offset(TOOL_KNIFE);
			/// yellow knife
			obj1_table.position.x = -0.56;
			obj1_table.position.y = 0.408-0.02;
			obj1_table.position.z = 0.19;
			obj1_table.orientation.x = -0.70710678118;
			obj1_table.orientation.y = 0.70710678118;
			obj1_table.orientation.z = 0;
			obj1_table.orientation.w = 0;
			/// bread cutter
  // 		      obj1_table.position.x = -0.580-0.05-0.02;
  // 		      obj1_table.position.y = 0.528-0.09;
  // 		      obj1_table.position.z = 0.19+0.03;
			obj2_angle = -90;
			is_obj2_round = false;
			set_orientation(obj2_table,obj2_angle,0,-180);
			//obj2_table.position.x = -0.148- 0.05;
			obj2_table.position.x = -0.2;
			obj2_table.position.y = 0.6-0.02-0.05 +0.03;
			obj2_table.position.z = 0.0;
// 			obj2_table.position.x = -0.148;
// 			obj2_table.position.y = 0.53;
// 			obj2_table.position.z = 0.023;
			break; 
		      }
		      /// unload
		      case 8: 
		      {
			obj1_table.position.x = -0.14-0.05;
			obj1_table.position.y = 0.514-0.02-0.08;
			obj1_table.position.z = 0.09;
			if(short_demo)
			{
			  obj1_table.position.x = -0.6;
			  obj1_table.position.y = 0.537;
			  obj1_table.position.z = 0.09;
// 			  obj1_table.position.x = -0.6;
// 			  obj1_table.position.y = 0.4;
// 			  obj1_table.position.z = 0.09;
			}
			set_orientation(obj1_table,-90,0,180);
			obj2_table.position.x = 0.2+0.05;
			obj2_table.position.y = 0.4+0.07-0.07;
			/// white bowl
			obj2_table.position.z = 0.30;
			/// blue bowl
// 			obj2_table.position.z = 0.27;
// 			obj2_table.position.y += 0.02;
			if(short_demo)
			{
			  obj2_table.position = bowl_pose.position;
			  obj2_table.position.x += 0.06;
			  obj2_table.position.y -= 0.1;
			  obj2_table.position.z += 0.02;
			}
			set_orientation(obj2_table,-90,0,180);
			goal_table2 = obj2_table;
			goal_table2.position.z -= 0.02;
			if(short_demo)
			  set_orientation(goal_table2,-90,0,-145);
			else
			{
			  //set_orientation(goal_table2,-90,0,-125);
			  set_orientation(goal_table2,-90,0,-135);
			}
			
			break;
		      }
		      
		      /// pour
		      case 18: 
		      {
			obj1_table.position.x = -1*0.005-0.04;
			obj1_table.position.y = 0.230+0.06;
			obj1_table.position.y += 0.03;
			obj1_table.position.z = 0.085;
			
			
			//set_orientation(obj1_table,-120,84,-98);
  // 		      set_orientation(obj1_table,-180,90,-140);
// 			set_orientation(obj1_table,-100,90,-80);
// 			set_orientation(obj1_table,-140,90,-180);
			//set_orientation(obj1_table,140,88,177);
			//set_orientation(obj1_table,70,-90,100);
			set_orientation(obj1_table,55,-90,100);
			if(short_demo)
			{
			  obj1_table.position.x = -0.1-0.4 -0.1+0.1+0.15+0.2;
			  obj1_table.position.y = 0.230+ 0.25-0.2-0.09;
			  obj1_table.position.z = 0.085;
			  set_orientation(obj1_table,-160,90,-180);
			}
			//log(Warning)<<"perception::set_pose >> Salad scenraio: obj1 orientation= "<<obj1_table.orientation<<endlog();
			obj2_table.position.x = 0.13;
			obj2_table.position.y = 0.45;
			obj2_table.position.y += 0.12;
			obj2_table.position.z = 0.25;
  // 		      set_orientation(obj2_table,-125,-5,-88);
			//set_orientation(obj2_table,-90,10,-90);
			set_orientation(obj2_table,90,0,90);
			//set_orientation(obj2_table,90,10,90);
			/// new orientation
			//set_orientation(obj1_table,177,88,177);
			//set_orientation(obj2_table,90,10,90);
			///
			if(short_demo)
			{
  // 			obj2_table.position.x = -0.24-0.05;
  // 			obj2_table.position.y = 0.50;
  // 			obj2_table.position.z = 0.331;
			  obj2_table.position =bowl_pose.position;
			  obj2_table.position.y -= 0.1;
			  set_orientation(obj2_table,-90,-0,-90);
			}
			
			//log(Warning)<<"perception::set_pose >> Salad scenraio: obj2 orientation= "<<obj2_table.orientation<<endlog();
// 			goal_table2.position = obj2_table.position;
			//goal_table2.position.x = 0.04;
			//goal_table2.position.y = 0.350;
// 			goal_table2.position.z += 0.22;
  // 		      set_orientation(goal_table2,-125,-45,-88);
// 			set_orientation(goal_table2,-90,-50,-140);
// 			set_orientation(goal_table2,-90,-50,-90);
			if(short_demo)
			{
			  set_orientation(goal_table2,-90,-60,-90);
			}
			//set_orientation(goal_table2,-135,-60,-80);
			//set_orientation(goal_table2,-128,-40,-80);
			//log(Warning)<<"perception::set_pose >> Salad scenraio: goal2 orientation= "<<goal_table2.orientation<<endlog();
			/// setting goal3 pose
// 			goal_table3 = goal_table2;
			if(short_demo)
			{
			  set_orientation(goal_table3,-90,-10,-90);
			}
			else
			{
  // 			set_orientation(goal_table3,-125,-5,-88);
// 			  set_orientation(goal_table3,-90,-10,-90);
			}
  // 		      goal_table3.position.x += 0.1;
  // 		      goal_table3.position.y -= 0.1;
// 			goal_table3.position.z += 0.05;
			
			///
// 			goal_table = obj1_table;
			//set_orientation(goal_table,-90,0,-180);
			//goal_table.position = obj1_table.position;
			//goal_table.position.x -= 0.1;
// 			set_orientation(goal_table,-180,90,-160);
			if(short_demo)
			{
			  goal_table.position.x += 0.1;
			  goal_table.position.y += 0.1;
  // 			goal_table.position.z -= 0.04;
			  
			  
  // 			goal_table.orientation.x = -0.70710678118;
  // 			goal_table.orientation.y = 0.70710678118;
  // 			goal_table.orientation.z = 0;
  // 			goal_table.orientation.w = 0;
			  set_orientation(goal_table,-180,90,-160);
  // 			set_orientation(goal_table,-90,-10,-90);
			}
			//goal_table.position.z = 0.1;
			//goal_table.position.x -=  0.15;
			//goal_table.position.y +=  0.05;
			//goal_table.position.x = 0.005;
			//goal_table.position.y =  0.23;
			//goal_table.position.z =  0.1;
			//goal_table.position.z =  obj1_table.position.z + 0.05;
			
			break;
		      }
		      ///stir
		      case 9: 
		      {
  // 		      obj1_table.position.x = 0.975;
  // 		      obj1_table.position.y = 0.313;
  // 		      obj1_table.position.z = 0.415-hand_offset.data[2];
// 			obj1_table.position.x = -0.580-0.05-0.22;
// 			obj1_table.position.y = 0.528-0.09;
// 			obj1_table.position.z = 0.19;
                        obj1_table.position.x = -0.63;
			obj1_table.position.y = 0.72;
			obj1_table.position.z = 0.19;
			obj1_table.orientation.x = -0.70710678118;
			obj1_table.orientation.y = 0.70710678118;
			obj1_table.orientation.z = 0;
			obj1_table.orientation.w = 0;
			if(short_demo)
			{
// 			  obj1_table.position.x = 0.16;//-0.580-0.05-0.22+0.34;
// 			  obj1_table.position.y = 0.5;//0.528-0.09+0.15;
// 			  obj1_table.position.z = 0.19;
			  /// spoon
			  obj1_table.position.x = -0.035;
			  obj1_table.position.y = 0.697;
			  obj1_table.position.z = 0.19;
			  /// knife
			  obj1_table.position.x = 0.158;
			  obj1_table.position.y = 0.513;
			  obj1_table.position.z = 0.19;
			  set_orientation(obj1_table,-150,0,-180);
			}
			
			
			obj2_table = obj1_table;
			obj2_table.position.x = 0.07+0.1+0.03;
			obj2_table.position.y = 0.464+0.1-0.05+0.05;
			obj2_table.position.z = 0.04;
			if(short_demo)
			{
  // 			obj2_table.position.x = -0.24-0.05;
  // 			obj2_table.position.y = 0.50;
			  obj2_table.position.x =bowl_pose.position.x;
// 			  obj2_table.position.y =bowl_pose.position.y+ 0.2;
  // 			obj2_table.position.z = 0.231;
  // 			obj2_table.position.x = -0.4;//0.07+0.1;
  // 			obj2_table.position.y = 0.464+0.05;
			  obj2_table.position.z = 0.05;
			  obj2_table.position.y -= 0.05;
			  obj2_table.position.x += 0.05; 
			}
			break;
		      }
  // 		    case 2: 
  // 		    {
  // 		      
  // 		      break;
  // 		    }
		      default:
		      {
			//log(Warning)<<"perception::set_pose >> Salad scenraio: Unknown action selected! "<<endlog();
		      }
		    } // end switch
		  } // end if(fixed_poses_salad_demo)
		  ///
		  /// depending on the action_type, define proper abstract points: home, goal etc.
		  switch(action_type)
		  {
		    case 1: /// pick and place
		    {
		      /// temp:
// 		      obj1_table.position.x = 0.518;
// 		      obj1_table.position.y = 0.591;
// 		      obj1_table.position.z = 0.0;
// 		      obj1_table.orientation.x = -0.70710678118;
// 		      obj1_table.orientation.y = 0.70710678118;
// 		      obj1_table.orientation.z = 0;
// 		      obj1_table.orientation.w = 0;
		      /// home
		      home_table = obj1_table;
		      home_table.position.z += 0.1; // move upwards
		      ///goal
		      float goal_shift_x, goal_shift_y, goal_shift_length;
		      goal_shift_length = 0.1;
		      float free_direction_norm=0;
		      if(free_direction_vector.size()>=3)
		      {
			free_direction_norm = free_direction_vector.at(0)*free_direction_vector.at(0)
			  + free_direction_vector.at(1)*free_direction_vector.at(1)
			  + free_direction_vector.at(2)*free_direction_vector.at(2);
		      }
		      //log(Error)<<"perception >> case 1 free_direction_vector size="<<free_direction_vector.size()
		       // <<"\tfree_direction_norm= "<<free_direction_norm<<endlog();
		      
		      if(is_free_direction_set&& std::abs(free_direction_norm)>0.8)
		      {
			goal_shift_x= goal_shift_length*free_direction_vector.at(0);
			goal_shift_y= goal_shift_length*free_direction_vector.at(1);
		      }
		      else
		      {
			goal_shift_x = 0.1;
			goal_shift_y = 0;
		      }
		      
		      goal_table = obj1_table;
		      goal_table.position.x += goal_shift_x;
		      goal_table.position.y += goal_shift_y;
		      goal_table.position.z = obj1_table.position.z - 0.04; // aim at a bit lower than original obj place
		      if (fixed_simple_demo)
		      {
			goal_table = obj1_table;
			goal_table.position = p3.position;
			goal_table.position.z -= 0.01;
		      }
			
		      minimum_height = 0.01;
		      if(goal_table.position.z <minimum_height)
		      {
		        //log(Warning)<<"perception::set_pose >> goal_table is too low, using minimum allowed height "<<endlog();
			goal_table.position.z = minimum_height;
		      }
// 		      set_orientation(goal_table,20,0,-180);
		      goal_table2 = goal_table;
		      goal_table2.position.z += 0.07;
		      break;
		    }
		    case 2: /// put on top
		    {
		      /// exhibition demo -- putting cups on bars
// 		      obj1_table.position.x = -0.39;
// 		      obj1_table.position.y = 0.66;
// 		      obj1_table.position.z = 0.015;
// 		      obj2_table.position.x = -0.1;
// 		      obj2_table.position.y = 0.73;
// 		      obj2_table.position.z = 0.1;
// 		      set_orientation(obj1_table,-171,82,155);
// 		      if(second_putontop)
// 		      {
// 			obj2_table.position.y -= 0.12;
// 			/// yellow cup
// 			obj1_table.position.x = -0.36;
// 			obj1_table.position.y = 0.48;
// 			obj1_table.position.z = 0.025;
// 			set_orientation(obj1_table,-178,80,177);
// 		      }
// 		      second_putontop = true;
// 		      
// 		      set_orientation(obj2_table,-178,88,177);
// 		      is_obj1_round = false;
		      /// icra pick and place experiment
// 		      obj1_table.position.x +=0.04;
// 		      obj1_table.position.x = -0.082;
// 		      obj1_table.position.y = 0.490;
// 		      obj1_table.position.z = 0.052;
// 		      set_orientation(obj1_table,-90,0,165);

		      /// temp: board
// 		      is_obj1_round = true;
// 		      obj1_table.position.x = -0.14-0.05;
// 		      obj1_table.position.y = 0.514-0.02-0.08;
// 		      obj1_table.position.z = 0.09;
// 		      obj2_table.position.x += 0.1;
		      /// cucumber on board
// 		      set_orientation(obj1_table,-90,0,-180);
// 		      is_obj1_round = false;
// 		      obj1_table.position.x = -0.4- 0.02;
// 		      obj1_table.position.y = 0.74-0.07/*-0.28*/;
// 		      obj1_table.position.z = 0.025;
// 		      obj2_table = obj1_table;
// 		      obj2_table.position.x = -0.145- 0.05/* -0.02*/;
// 		      obj2_table.position.y = 0.6-0.03 - 0.05;
// 		      obj2_table.position.z = 0.025;
// 		      if(!is_obj1_round)
// 			set_orientation(obj1_table,obj1_angle-180,0,-180);
		      /// home position
		      home_table = obj1_table;
		      home_table.position.z = obj1_table.position.z + 0.1; // move upwards
// 		      home_table.position.x = obj1_table.position.x + 0.05; 
		      /// obj2 adjust
		      //obj2_table.position.y += 0.03; // adjustment for the cutting board
		      /// Goal position

		      /// for ICRA demo
// 		      goal_table2 = obj2_table;
// 		      goal_table2.position.z = home_table.position.z -0.15;
		      /// normal
		      goal_table2 = obj2_table;
		      goal_table2.position.z = 2*obj2_table.position.z + 0.1; // above the obj2
		      
		      break;
		    }
		    case 3: /// take down
		    {
		      /// 
		      if (obj1_table.position.z>0.06)
			obj1_table.position.z=0.06;
		      /// home position
		      home_table = obj1_table;
		      home_table.position.z = obj1_table.position.z + 0.05; // move upwards
		      /// Goal position
		      // a random goal position.
		      // select a random angle between 0 and 180 degrees
		      float rand_integer = std::rand() % 180;
		      float rand_angle = (float) rand_integer ;
		      float goal_distance = 0.2;
		      float goal_x_distance = goal_distance * cos(3.1415* rand_angle / 180);
		      float goal_y_distance = goal_distance * sin(3.1415* rand_angle / 180);
		      goal_table = obj1_table;
// 		      goal_table.position.x += goal_x_distance;
// 		      goal_table.position.y += goal_y_distance;
		      
		      goal_table.position.x += 0.2;//-0.200;//-0.244;//-0.195;
		      goal_table.position.y += 0.05;//0.480;//0.405;//0.485;
		      if(fixed_simple_demo)
		      {
			goal_table.position.x = p1.position.x;
			goal_table.position.y = p1.position.y;
		      }
		      /// goal is table
		      goal_table.position.z = 0.0;//0.095;//0.366-0.065;
		      
		      
		      break;
		    }
		    case 4: /// hide_in
		    {
		      /// home position
		      home_table = obj1_table;
		      home_table.position.z = obj1_table.position.z + 0.15; // move upwards
		      /// Goal position
		      obj2_table.position.z += 0.05;
		      goal_table = obj2_table;
		      goal_table.position.z = obj2_table.position.z + 0.08;
		      if(!is_obj1_round)
			set_orientation(obj1_table,obj1_angle-180,0,-180);
		      break;
		    }
		    case 5: /// grasp tool
		    {
		      /// home position
		      /// for testing: overwrite the obj1 pose with some predefined values
// 		      goal_x = cos(M_PI/6.0)*0.232 - sin(M_PI/6.0)*0.227;
// 		      goal_y = sin(M_PI/6.0)*0.232 + cos(M_PI/6.0)*0.227;
		      obj1_table = obj1_camera;
// 		      obj1_msgs_robot = obj1_table;
// 		      obj1_msgs_robot.position.x = cos(M_PI/6.0)*obj1_table.position.x - sin(M_PI/6.0)*obj1_table.position.y;
// 		      obj1_msgs_robot.position.y = sin(M_PI/6.0)*obj1_table.position.x + cos(M_PI/6.0)*obj1_table.position.y;
// 		      obj1_msgs_robot.position.z = obj1_table.position.z;
		      ///
		      home_table = obj1_table;
		      home_table.position.z += 0.10;
		      /// Goal position (not used in this action)
		      goal_table = obj1_table;
		      goal_table.position.z = obj1_table.position.z - 0.02; // aim at a bit lower than original obj place
		      break;
		    }
		    case 6: /// cut
		    {
		      /// obj1 orientation
// 		      float angle_a, angle_b, angle_c;
// 		      angle_a = obj1_angle + 30*M_PI/180; // 30 degree is added to convert from Table frame to Robot frame
// 		      angle_b = 0;
// 		      angle_c = -100*M_PI/180;
// 		      KDL::Rotation rot1= KDL::Rotation::EulerZYX(angle_a,angle_b,angle_c);
// 		      KDL::Frame frame1;
// 		      frame1.M = rot1;
// 		      geometry_msgs::Pose pos1;
// 		      tf::poseKDLToMsg(frame1,pos1);
// 		      obj1_msgs_robot.orientation = pos1.orientation;
// 		      if(!is_obj1_round)
		      //log(Warning)<<"perception::set_pose >> case cutting: obj1_angle= "<<obj1_angle;
// 		      float knife_angle = obj1_angle;
		      float knife_angle = -45;
		      /// knife angle must be between 0 and 180 degrees
// 		      if(knife_angle<0)
// 			knife_angle += 180;
// 		      else if(knife_angle>180)
// 			knife_angle -= 180;
		      set_orientation(obj1_table,knife_angle,0,-90);
		      //log(Warning)<<"perception::set_pose >> knife_angle= "<<knife_angle<<endlog();
		      /// angle for sawing movement, used in ARM_EXERT state
		      float saw_angle;
		      if(knife_angle<90)
			saw_angle = knife_angle + 90;
		      else
			saw_angle = knife_angle - 90;
		      //log(Warning)<<"perception::set_pose >> saw_angle= "<<saw_angle<<endlog();
		      knife_angle_cosine = cos(M_PI*saw_angle/180);
		      knife_angle_sine = sin(M_PI*saw_angle/180);
		      /// home position
		      home_table = obj1_table;
		      home_table.position.z += 0.15;
		      /// Goal position (not used in this action)
		      goal_table = obj1_table;
		      /// with force control
		      goal_table.position.z = obj1_table.position.z + 0.12; // aim higher than original obj place
		      /// without force control
// 		      goal_table.position.z = 0; // aim higher than original obj place
		      break;
		    }
		    case 7: /// release tool
		    {
		      
		      /// home position
// 		      home_table = obj1_msgs_robot;
// 		      home_table.position.z += 0.10;
		      /// Goal position
		      float angle_a, angle_b, angle_c;
		      angle_a = 50*M_PI/180;
		      angle_b = 0;
		      angle_c = -100*M_PI/180;
		      KDL::Rotation rot1= KDL::Rotation::EulerZYX(angle_a,angle_b,angle_c);
		      KDL::Frame frame1;
		      frame1.M = rot1;
		      
		      frame1.p.data[0] = -0.55;
		      frame1.p.data[1] = 0.520;
		      frame1.p.data[2] = 0.05;
		      geometry_msgs::Pose pos1;
		      tf::poseKDLToMsg(frame1,pos1);
		      goal_table = pos1;
		      home_table = goal_table;
		      home_table.position.z += 0.10;
// 		      goal_table.position.z = obj1_msgs_robot.position.z - 0.02; // aim at a bit lower than original obj place
// 		      //log(Warning)<<"perception::set_pose >> home pose for action 7 is set to= "<<home_table.position.x<<" "
// 		        <<home_table.position.y<<" "<<home_table.position.z<<endlog();
		      break;
		    }
		    case 8: /// unload
		    {
		      /// for now, obj1 pose is not from vision, it is fixed here
// 		      obj1_table.position.x = 0.36;
// 		      obj1_table.position.y = 0.597;
// 		      obj1_table.position.z = 0.31 - hand_offset.data[2];

		      /// "fixing" the obj2 pose
// 		      obj2_table.position.z += 0.22;//0.15;
// // 		      obj2_table.position.x -= 0.20;//0.05;
// 		      obj2_table.position.y -= 0.15;//0.05;
		      

		      float angle_a, angle_b, angle_c;
// 		      angle_a = 120*M_PI/180;
// 		      angle_b = 0*M_PI/180;
// 		      angle_c = 155*M_PI/180;
		      angle_a = -90*M_PI/180;
		      angle_b = 0*M_PI/180;
		      angle_c = -140*M_PI/180;
		      KDL::Rotation rot1= KDL::Rotation::EulerZYX(angle_a,angle_b,angle_c);
		      KDL::Frame frame1;
		      frame1.M = rot1;
		      
		      //float x_table, y_table;
		      //x_table = obj2_table.position.x;
		      //y_table = obj2_table.position.y;
		      frame1.p.data[0] = obj2_table.position.x;//cos(M_PI/6.0)*x_table - sin(M_PI/6.0)*y_table;
		      frame1.p.data[1] = obj2_table.position.y;//sin(M_PI/6.0)*x_table + cos(M_PI/6.0)*y_table;
		      frame1.p.data[2] = obj2_table.position.z;
		      geometry_msgs::Pose pos1;
		      tf::poseKDLToMsg(frame1,pos1);
		      //goal_table2 = pos1;
// 		      float goal_x,goal_y;
// 		      float goal_x_table = obj2_table.position.x;
// 		      float goal_y_table = obj2_table.position.y-0.20;
// 		      goal_x = cos(M_PI/6.0)*goal_x_table - sin(M_PI/6.0)*goal_y_table;
// 		      goal_y = sin(M_PI/6.0)*goal_x_table + cos(M_PI/6.0)*goal_y_table;
// 		      obj2_msgs_robot.position.z += 0.15;
// 		      obj2_msgs_robot.position.x = goal_x;
// 		      obj2_msgs_robot.position.y = goal_y;
		      /// setting goal pose
		      
		      goal_table = obj1_table;
		      if(short_demo)
		      {
			goal_table.position.x-=0.03;
			goal_table.position.y+=0.1;
		      }
		      else
		      {
		        goal_table.position.x-=0.1;
		        goal_table.position.y+=0.1+0.08;
		      }
		      
		      goal_table.position.z = obj1_table.position.z - 0.02; // aim at a bit lower than original obj place
		      break;
		    }
		    case 9: /// stirring
		    {
		      /// obj1 position
// 		      obj1_table.position.x = -0.010;
// 		      obj1_table.position.y = 0.515;
// 		      obj1_table.position.z = 0.4-0.23;
// 		      obj1_table.position.x = 0.975;
// 		      obj1_table.position.y = 0.313;
// 		      obj1_table.position.z = 0.415-hand_offset.data[2];
// 		      obj1_table.orientation.x = -0.70710678118;
// 		      obj1_table.orientation.y = 0.70710678118;
// 		      obj1_table.orientation.z = 0;
// 		      obj1_table.orientation.w = 0;
		      
		      //obj1_table.position.z += 0.05;
		      // home
		      home_table = obj1_table;
		      home_table.position.z -= 0.05;
		      // obj2
// 		      obj2_table.position.z += 0.1;
		      home2_table = obj2_table;
		      /// blue bowl
		      //home2_table.position.z += 0.15-0.03;
		      /// white bowl
		      home2_table.position.z += 0.17;
		      home2_table.position.z += 0.02;
		      
		      //home2_table.position.x -= 0.05;
		      //home2_table.position.y -= 0.03;
		      // goal
		      goal_table = obj1_table;
// 		      goal_table.position.x -= 0.1;
// 		      goal_table.position.y += 0.1;
		      goal_table.position.z = 0.08;
		      
		      

		      break;
		    }
		    case 10: /// cut2
		    {
		      /// phase 1 : grasping the knife
		      /// home position
// 		      base_knife.position.x = x_coeff*0.242;
//       base_knife.position.y = 0.446;
//       base_knife.position.z = 0.20;
		      int arm_coeff = 1;
		      if(arm_number==1)
		      {
			arm_coeff = -1;
		      }
		      else if(arm_number==0)
		      {
			arm_coeff = 1;
			
		      }
		      else
		      {
			log(Error)<<"perception::set_pose >> Invalid arm_number is selected= "<<arm_number<<endlog();
		      }
		      /// yellow knife
			obj1_table.position.x = -0.56;
			obj1_table.position.y = 0.408-0.02;
			obj1_table.position.z = 0.19;
			obj1_table.orientation.x = -0.70710678118;
			obj1_table.orientation.y = 0.70710678118;
			obj1_table.orientation.z = 0;
			obj1_table.orientation.w = 0;
			/// bread cutter
  // 		      obj1_table.position.x = -0.580-0.05-0.02;
  // 		      obj1_table.position.y = 0.528-0.09;
  // 		      obj1_table.position.z = 0.19+0.03;
			obj2_angle = -90;
			is_obj2_round = false;
			set_orientation(obj2_table,obj2_angle,0,-180);
			//obj2_table.position.x = -0.148- 0.05;
			obj2_table.position.x = -0.2;
			obj2_table.position.y = 0.6-0.02-0.05 +0.03;
			obj2_table.position.z = 0.0;
		      // for arm_number=1
// 		      obj1_table.position.x = arm_coeff*0.580;
// 		      obj1_table.position.y = 0.528-0.09;
// 		      obj1_table.position.z = 0.19;
		      // for arm_number=0
		      //obj1_table.position.x = arm_coeff*0.728;
		      //obj1_table.position.y = 0.478;
		      //obj1_table.position.z = 0.408- hand_offset.data[2];
// 		      
// 		      obj1_table.orientation.x = 1;
// 		      obj1_table.orientation.y = 0;
// 		      obj1_table.orientation.z = 0;
// 		      obj1_table.orientation.w = 0;
// 		      obj1_table.orientation.x = -0.70710678118;
// 		      obj1_table.orientation.y = 0.70710678118;
// 		      obj1_table.orientation.z = 0;
// 		      obj1_table.orientation.w = 0;
// 		      goal_table2 = obj1_table;
// 		      goal_table2.position.z += 0.04 + knife_offset.data[2] - hand_offset.data[2]+0.19 - obj1_table.position.z-0.1;
		      
		      home_table = obj1_table;
		      home_table.position.z += 0.04 + knife_offset.data[2] - hand_offset.data[2]+0.19 - obj1_table.position.z-0.1;
// 		      home_table.position.x -= 0.2;
// 		      home_table.position.y += 0.05;
// 		      home_table.position.z += 0.0;
		      
		      /// obj2 orientation
// 		      float angle_a, angle_b, angle_c;
// 		      angle_a = obj1_angle + 30*M_PI/180; // 30 degree is added to convert from Table frame to Robot frame
// 		      angle_b = 0;
// 		      angle_c = -100*M_PI/180;
// 		      KDL::Rotation rot1= KDL::Rotation::EulerZYX(angle_a,angle_b,angle_c);
// 		      KDL::Frame frame1;
// 		      frame1.M = rot1;
// 		      geometry_msgs::Pose pos1;
// 		      tf::poseKDLToMsg(frame1,pos1);
// 		      obj1_msgs_robot.orientation = pos1.orientation;
// 		      if(!is_obj1_round)
		      //log(Warning)<<"perception::set_pose >> case cutting2: obj2_angle= "<<obj2_angle<<endlog();
		      float y_middle = 0.5;
		      float knife_angle = obj2_angle;
		      if(obj2_table.position.y>=y_middle)
		      {
			//log(Warning)<<"perception::set_pose >> target is far, knife angle is adjusted to interval [-90 90] "<<endlog();
			if(knife_angle>90)
			  knife_angle -= 180;
			else if(knife_angle<-90)
			  knife_angle += 180;
		      }
		      else // obj2_table.position.y<y_middle
		      {
			//log(Warning)<<"perception::set_pose >> target is close, knife angle is adjusted to interval [-180 -90] or [90 180] "<<endlog();
			if(std::abs(knife_angle)<90)
			{
			  if(knife_angle>0)
			    knife_angle -=180;
			  else if(knife_angle<0)
			    knife_angle +=180;
			}
		      }
		      //log(Warning)<<"perception::set_pose >> knife_angle= "<<knife_angle<<endlog();
		      //knife_angle +=180;
		      //float knife_angle = -45;
		      /// knife angle must be between 0 and 180 degrees
// 		      if(knife_angle<0)
// 			knife_angle += 180;
// 		      else if(knife_angle>180)
// 			knife_angle -= 180;
		      /// calculate the euler angles
		      std::vector<double> euler_angles = getZYXEulerfromPose(obj1_table);
		      double delta_phi;
		      if(false&&euler_angles.at(0)<=180 && euler_angles.at(0)>=90)
		      {
			
			delta_phi = euler_angles.at(0)- 270;
			//log(Warning)<<"perception::set_pose >> 3rd quadrant. delta_phi= "<<delta_phi<<endlog();
		      }
		      else
		      {
			delta_phi = euler_angles.at(0) + 90;
			//log(Warning)<<"perception::set_pose >> delta_phi= "<<delta_phi<<endlog();
		      }
		      double corrected_theta = -1* delta_phi;
		      set_orientation(obj2_table,knife_angle,corrected_theta,-100);
		      
		      /// angle for sawing movement, used in ARM_EXERT state
		      float saw_angle = knife_angle;
		      if(knife_angle<90)
			saw_angle = knife_angle + 90;
		      else
			saw_angle = knife_angle - 90;
		      
		      //log(Warning)<<"perception::set_pose >> saw_angle= "<<saw_angle<<endlog();
		      knife_angle_cosine = cos(M_PI*saw_angle/180);
		      knife_angle_sine = sin(M_PI*saw_angle/180);
		      //log(Warning)<<"perception::set_pose >> knife_angle_cosine= "<<knife_angle_cosine<<endlog();
		      //log(Warning)<<"perception::set_pose >> knife_angle_sine= "<<knife_angle_sine<<endlog();
		      /// phase 2 : cutting
		      /// home2 position
		      /// for now, set the obj2_table z position to zero
// 		      obj2_table.position.z = 0;
		      home2_table = obj2_table;
		      home2_table.position.z = 0.14;
		      
		      /// Phase 3: put the knife on the table
		      /// Goal position
// 		      float angle_a, angle_b, angle_c;
// 		      angle_a = 50*M_PI/180;
// 		      angle_b = 0;
// 		      angle_c = -100*M_PI/180;
// 		      KDL::Rotation rot1= KDL::Rotation::EulerZYX(angle_a,angle_b,angle_c);
// 		      KDL::Frame frame1;
// 		      frame1.M = rot1;
// 		      
// 		      frame1.p.data[0] = -0.55;
// 		      frame1.p.data[1] = 0.520;
// 		      frame1.p.data[2] = 0.05;
// 		      geometry_msgs::Pose pos1;
// 		      tf::poseKDLToMsg(frame1,pos1);
		      goal_table.orientation = obj2_table.orientation;
		      goal_table.position = obj2_table.position;
		      goal_table.position.x -=0.25;
		      goal_table.position.y -=0.12;
		      goal_table.position.z =0.03;
		      /// with force control
// 		      goal_table.position.z = obj1_table.position.z + 0.12; // aim higher than original obj place
		      /// without force control
// 		      goal_table.position.z = 0; // aim higher than original obj place
		      /// move the cut pieces to the side
		      goal_table3 = obj2_table;
		      goal_table3.position.y +=0.02;
		      break;
		    }
		    case 11: /// push
		    {  
		      
		      /// Goal position
		      goal_table = obj1_table;
		      goal_table.position.y += 0.1;
		      /// home position
		      // obj1 position should lie between home and goal2
		      // obj1 = alpha*home + (1-alpha)*goal
		      // alpha = dist(home,obj1)/[dist(home,obj1),dist(obj1,goal)]
		      home_table = obj1_table;
		      float dist_home_obj1 = 0.1; //fixed
		      
		      float alpha = dist_home_obj1/(dist_home_obj1+distance_xy(obj1_table,goal_table));
		      //log(Warning)<<"perception::set_pose >> alpha is= "<<alpha<<endlog();
		      home_table.position.x = (1/alpha)*obj1_table.position.x + (alpha-1)*goal_table.position.x/alpha;
		      home_table.position.y = (1/alpha)*obj1_table.position.y + (alpha-1)*goal_table.position.y/alpha;
		      
// 		      if(!is_obj1_round)
// 			set_orientation(obj1_table,obj1_angle-180,0,-180);
		      break;
		    }
		    case 12: /// push with grasp
		    {  
		      obj1_table.position.z =0.07;
		      
		      /// Goal position
		      ///goal
		      float goal_shift_x, goal_shift_y, goal_shift_length;
		      goal_shift_length = 0.2;
		      float free_direction_norm=0;
		      if(free_direction_vector.size()>=3)
		      {
			free_direction_norm = free_direction_vector.at(0)*free_direction_vector.at(0)
			  + free_direction_vector.at(1)*free_direction_vector.at(1)
			  + free_direction_vector.at(2)*free_direction_vector.at(2);
		      }
		      //log(Error)<<"perception >> case 12 free_direction_vector size="<<free_direction_vector.size()
		       // <<"\tfree_direction_norm= "<<free_direction_norm<<endlog();
		      
		      if(is_free_direction_set&& std::abs(free_direction_norm)>0.8)
		      {
			goal_shift_x= goal_shift_length*free_direction_vector.at(0);
			goal_shift_y= goal_shift_length*free_direction_vector.at(1);
		      }
		      else
		      {
			goal_shift_x = 0.1;
			goal_shift_y = 0;
		      }
		      /// relative to obj1
		      //goal_table = obj1_table;
		      //goal_table.position.x += goal_shift_x;
		      //goal_table.position.y += goal_shift_y;
		      /// between obj1 and obj2
		      goal_table = obj1_table;
		      float alfa = 0.2;
		      goal_table.position.x = alfa * obj1_table.position.x + (1-alfa) * obj2_table.position.x;
		      goal_table.position.y = alfa * obj1_table.position.y + (1-alfa) * obj2_table.position.y;

		      break;
		    }
		    case 13: /// poke
		    {  
		      
		      /// Home position
		      home_table = obj1_table;
		      home_table.position.z += 0.07;
		      

		      break;
		    }
		    case 14: /// push by holding
		    {  
		      
		      /// Home position
		      home_table = obj1_table;
		      home_table.position.z += 0.07;
		      ///goal
		      float goal_shift_x, goal_shift_y, goal_shift_length;
		      goal_shift_length = 0.2;
		      float free_direction_norm=0;
		      if(free_direction_vector.size()>=3)
		      {
			free_direction_norm = free_direction_vector.at(0)*free_direction_vector.at(0)
			  + free_direction_vector.at(1)*free_direction_vector.at(1)
			  + free_direction_vector.at(2)*free_direction_vector.at(2);
		      }
		      //log(Error)<<"perception >> case 14 free_direction_vector size="<<free_direction_vector.size()
		       // <<"\tfree_direction_norm= "<<free_direction_norm<<endlog();
		      
		      if(is_free_direction_set&& std::abs(free_direction_norm)>0.8)
		      {
			goal_shift_x= goal_shift_length*free_direction_vector.at(0);
			goal_shift_y= goal_shift_length*free_direction_vector.at(1);
		      }
		      else
		      {
			goal_shift_x = 0.1;
			goal_shift_y = 0;
		      }
		      goal_table = obj1_table;
		      goal_table.position.x += goal_shift_x;
		      goal_table.position.y += goal_shift_y;
// 		      goal_table = obj1_table;
// 		      goal_table.position.x -= 0.07;
// 		      goal_table.position.y -= 0.05;
		      home2_table = goal_table;
		      home2_table.position.z +=0.1;

		      break;
		    }
		    case 15: /// push apart by grasping
		    {  
		      
		      goal_table = obj1_table;
		      float X_goal,Y_goal;
		      calculate_goal_push_apart(X_goal,Y_goal,0.1);
		      goal_table.position.x = X_goal;
		      goal_table.position.y = Y_goal;

		      break;
		    }
		    case 16: /// push apart by holding
		    {  
		      
		      goal_table = obj1_table;
		      float X_goal,Y_goal;
		      calculate_goal_push_apart(X_goal,Y_goal,0.15);
		      goal_table.position.x = X_goal;
		      goal_table.position.y = Y_goal;
		      
		      home_table = obj1_table;
		      home_table.position.z += 0.07;
		      
		      home2_table = goal_table;
		      home2_table.position.z +=0.1;

		      break;
		    }
		    case 17: /// push together by holding
		    {  
		      
		      goal_table = obj2_table;
		      goal_table.position.z = obj1_table.position.z;
		      
		      
		      home_table = obj1_table;
		      home_table.position.z += 0.07;
		      
		      home2_table = goal_table;
		      home2_table.position.z +=0.1;

		      break;
		    }
		    case 18: /// pouring (from a bottle or jar)
		    {  
		      /// fixed position for bottle
// 		      obj1_table.position.x = -1*0.005-0.04;
// 			obj1_table.position.y = 0.230+0.06;
// 			obj1_table.position.y += 0.03;
// 			obj1_table.position.z = 0.085;
// 			obj2_table.position.x = 0.13;
			
// 			obj2_table.position.y += 0.12;
// 			obj2_table.position.z = 0.12;
// 			obj2_table.position.z = 0.15;
			//set_orientation(obj1_table,-120,84,-98);
  // 		      set_orientation(obj1_table,-180,90,-140);
// 			set_orientation(obj1_table,-100,90,-80);
// 			set_orientation(obj1_table,-140,90,-180);
// 			set_orientation(obj1_table,177,88,177);
			//set_orientation(obj1_table,140,88,177);
			
			//set_orientation(obj2_table,-90,-0,-90);
// 			set_orientation(obj2_table,90,10,90);
			if(short_demo)
			{
  // 			obj2_table.position.x = -0.24-0.05;
  // 			obj2_table.position.y = 0.50;
  // 			obj2_table.position.z = 0.331;
			  obj2_table.position =bowl_pose.position;
			  obj2_table.position.y -= 0.1;
			  set_orientation(obj2_table,-90,-0,-90);
			}
			
			//log(Warning)<<"perception::set_pose >> Salad scenraio: obj2 orientation= "<<obj2_table.orientation<<endlog();
			goal_table2.position = obj2_table.position;
			//goal_table2.position.x = 0.04;
			//goal_table2.position.y = 0.350;
			goal_table2.position.z += 0.02;
  // 		      set_orientation(goal_table2,-125,-45,-88);
			//set_orientation(goal_table2,-90,-50,-140);
			set_orientation(goal_table2,90,40,90);
			//set_orientation(goal_table2,90,-40,90);
			//set_orientation(goal_table2,-90,-40,-90);
			if(short_demo)
			{
			  set_orientation(goal_table2,-90,-60,-90);
			}
			//set_orientation(goal_table2,-135,-60,-80);
			//set_orientation(goal_table2,-128,-40,-80);
			//log(Warning)<<"perception::set_pose >> Salad scenraio: goal2 orientation= "<<goal_table2.orientation<<endlog();
			/// setting goal3 pose
			goal_table3 = goal_table2;
			if(short_demo)
			{
			  set_orientation(goal_table3,-90,-10,-90);
			}
			else
			{
			  //set_orientation(goal_table3,-90,-10,-90);
			  set_orientation(goal_table3,90,20,90);
			  //set_orientation(goal_table3,-90,-50,-90);
			}
  // 		      goal_table3.position.x += 0.1;
  // 		      goal_table3.position.y -= 0.1;
			goal_table3.position.z += 0.05;
			
			///
			goal_table = obj1_table;
			//set_orientation(goal_table,-90,0,-180);
			//goal_table.position = obj1_table.position;
			//goal_table.position.x -= 0.1;
			//set_orientation(goal_table,180,90,180);
// 			set_orientation(goal_table,180,-90,100);
			if(short_demo)
			{
			  goal_table.position.x += 0.1;
			  goal_table.position.y += 0.1;
  // 			goal_table.position.z -= 0.04;
			  
			  
  // 			goal_table.orientation.x = -0.70710678118;
  // 			goal_table.orientation.y = 0.70710678118;
  // 			goal_table.orientation.z = 0;
  // 			goal_table.orientation.w = 0;
  // 			set_orientation(goal_table,-90,-10,-90);
			}
		      /// setting the orientation for obj1 and obj2
		      float angle_a, angle_b, angle_c;
		      angle_a = 0;
		      angle_b = 80;
		      angle_c = 180;
		      //log(Warning)<<"perception::set_pose >> case 18: adjust obj1_table"<<endlog();
		      //set_orientation(obj1_table,angle_a,angle_b,angle_c);
// 		      //log(Warning)<<"perception::set_pose >> case 18: adjust goal_table"<<endlog();
// 		      set_orientation(goal_table,angle_a,angle_b,angle_c);
		      angle_a = -80;
		      angle_b = 45;
		      angle_c = 95;
		      //log(Warning)<<"perception::set_pose >> case 18: adjust obj2_table"<<endlog();
		      //set_orientation(obj2_table,angle_a,angle_b,angle_c);
		      angle_b = -35;
		      angle_c = 90;
		      //log(Warning)<<"perception::set_pose >> case 18: adjust goal_table2"<<endlog();
		      //set_orientation(goal_table2,angle_a,angle_b,angle_c);
		      
		      /// "fixing" the obj2 pose
		      //obj2_table.position.z += 0.20;//0.15;
// 		      obj2_table.position.x -= 0.20;//0.05;
		      //obj2_table.position.y -= 0.1;//0.05;
		      /// setting goal2 pose
		      //goal_table2.position = obj2_table.position;

		      /// setting goal pose
		      // fixing obj1 pose
		      //obj1_table.position.y -= 0.08;
		      //goal_table = obj1_table;
// 		      goal_table.position.x = goal_x;
// 		      goal_table.position.y = goal_y;
		      //goal_table.position.z = obj1_table.position.z - 0.02; // aim at a bit lower than original obj place
		      break;
		      

		    }
		    case 19: /// chopping
		    {
		      /// phase 1 : grasping the knife
		      /// home position
// 		      base_knife.position.x = x_coeff*0.242;
//       base_knife.position.y = 0.446;
//       base_knife.position.z = 0.20;
		      int arm_coeff = 1;
		      if(arm_number==1)
		      {
			arm_coeff = -1;
		      }
		      else if(arm_number==0)
		      {
			arm_coeff = 1;
			
		      }
		      else
		      {
			log(Error)<<"perception::set_pose >> Invalid arm_number is selected= "<<arm_number<<endlog();
		      }
// 		      obj1_table.position.x = arm_coeff*0.567;
// 		      obj1_table.position.y = 0.376+0.15;
// 		      obj1_table.position.z = 0.19;
		      obj1_table.position.x = arm_coeff*0.718;
		      obj1_table.position.y = 0.591;
		      obj1_table.position.z = 0.184;
		      float angle_a, angle_b, angle_c;
		      angle_a = 0;
		      angle_b = 0;
		      angle_c = -133;
		      //log(Warning)<<"perception::set_pose >> case 19: adjust obj1_table"<<endlog();
		      set_orientation(obj1_table,angle_a,angle_b,angle_c);
		      home_table = obj1_table;
		      home_table.position.z += 0.04 + cleaver_offset.data[2] - hand_offset.data[2];
		      
		      /// obj2 orientation
// 		      float angle_a, angle_b, angle_c;
// 		      angle_a = obj1_angle + 30*M_PI/180; // 30 degree is added to convert from Table frame to Robot frame
// 		      angle_b = 0;
// 		      angle_c = -100*M_PI/180;
// 		      KDL::Rotation rot1= KDL::Rotation::EulerZYX(angle_a,angle_b,angle_c);
// 		      KDL::Frame frame1;
// 		      frame1.M = rot1;
// 		      geometry_msgs::Pose pos1;
// 		      tf::poseKDLToMsg(frame1,pos1);
// 		      obj1_msgs_robot.orientation = pos1.orientation;
// 		      if(!is_obj1_round)
// 		      obj2_angle = 0;
		      //log(Warning)<<"perception::set_pose >> case chopping: obj2_angle= "<<obj2_angle<<endlog();
		      float y_middle = 0.5;
		      float knife_angle = obj2_angle;
		      if(obj2_table.position.y>=y_middle)
		      {
			//log(Warning)<<"perception::set_pose >> target is far, knife angle is adjusted to interval [-90 90] "<<endlog();
			if(knife_angle>90)
			  knife_angle -= 180;
			else if(knife_angle<-90)
			  knife_angle += 180;
		      }
		      else // obj2_table.position.y<y_middle
		      {
			//log(Warning)<<"perception::set_pose >> target is close, knife angle is adjusted to interval [-180 -90] or [90 180] "<<endlog();
			if(std::abs(knife_angle)<90)
			{
			  if(knife_angle>0)
			    knife_angle -=180;
			  else if(knife_angle<0)
			    knife_angle +=180;
			}
		      }
		      //log(Warning)<<"perception::set_pose >> knife_angle= "<<knife_angle<<endlog();
		      //knife_angle +=180;
		      //float knife_angle = -45;
		      /// knife angle must be between 0 and 180 degrees
// 		      if(knife_angle<0)
// 			knife_angle += 180;
// 		      else if(knife_angle>180)
// 			knife_angle -= 180;
		      set_orientation(obj2_table,knife_angle,0,-100);
		      
		      
		      /// phase 2 : cutting
		      /// home2 position
// 		      obj2_table.position.x = 0.5;
// 		      obj2_table.position.y = 1;
// 		      obj2_table.position.z = 0.04;
		      background_table = obj2_table;
		      background_table.position.z = 0;
		      home2_table = obj2_table;
		      home2_table.position.z += 0.14;
		      /// Phase 3: put the knife on the table
		      /// Goal position
// 		      float angle_a, angle_b, angle_c;
// 		      angle_a = 50*M_PI/180;
// 		      angle_b = 0;
// 		      angle_c = -100*M_PI/180;
// 		      KDL::Rotation rot1= KDL::Rotation::EulerZYX(angle_a,angle_b,angle_c);
// 		      KDL::Frame frame1;
// 		      frame1.M = rot1;
// 		      
// 		      frame1.p.data[0] = -0.55;
// 		      frame1.p.data[1] = 0.520;
// 		      frame1.p.data[2] = 0.05;
// 		      geometry_msgs::Pose pos1;
// 		      tf::poseKDLToMsg(frame1,pos1);
		      goal_table = obj2_table;
		      goal_table.position.x -=0.03;
		      goal_table.position.y +=0.10;
		      goal_table.position.z +=0.02;
		      /// with force control
// 		      goal_table.position.z = obj1_table.position.z + 0.12; // aim higher than original obj place
		      /// without force control
// 		      goal_table.position.z = 0; // aim higher than original obj place
		      break;
		    }
		  }
		  obj_set_outport.write(true);
		  is_perception_ready = true;
		  return true;
		}
		void Perception::initialize_relations()
		{
		  std::vector<std::string> init(6);
		  bool is_initial_fixed = true;
		  //log(Warning)<<"perception::initialize_relations >> initializing relations for action number= "<<action_type<<endlog();
		  /// first three are initialized to "N", assuming that initially the hand is empty and not touching other objects
		  if(relation_1_init!="fixed" && relation_2_init!="fixed" && relation_3_init!="fixed")
		  {
		    init.at(3)= relation_1_init; // obj1 obj2
		    init.at(4)= relation_2_init; // obj1 background
		    init.at(5)= relation_3_init; // obj2 background
		  }
		  else
		    is_initial_fixed = true;
		  switch(action_type)
		  {
		    
		    case 1: /// pick and place
		    case 5: /// grasp tool
		    case 6: /// cut
		    case 11: /// push
		    case 12: /// push with grasp
		    case 13: /// poke
		    case 14: /// push by holding
		      init.at(0)= "N"; // hand obj1
		      init.at(1)= "A"; // hand obj2
		      init.at(2)= "N"; // hand background
		      /// the last three relations should be initialized by Vision system.
		      if(is_initial_fixed)
		      {
		        init.at(3)= "A"; // obj1 obj2
		        init.at(4)= "T"; // obj1 background
		        init.at(5)= "A"; // obj2 background
		      }
		      
		      break;
		    case 2: /// put on top
		    case 4: /// hide_in
		    case 9: /// stir
		    case 10: /// cut2
		    case 19: /// chopping
		    case 17: /// push together by holding
		      init.at(0)= "N"; // hand obj1
		      init.at(1)= "N"; // hand obj2
		      init.at(2)= "N"; // hand background
		      /// the last three relations should be initialized by Vision system.
		      if(is_initial_fixed)
		      {
		        init.at(3)= "N"; // obj1 obj2
		        init.at(4)= "T"; // obj1 background
		        init.at(5)= "T"; // obj2 background
		      }
		      break;
		    case 3: /// take down
		      init.at(0)= "N"; // hand obj1
		      init.at(1)= "N"; // hand obj2
		      init.at(2)= "N"; // hand background
		      /// the last three relations should be initialized by Vision system.
		      if(is_initial_fixed)
		      {
		        init.at(3)= "T"; // obj1 obj2
		        init.at(4)= "N"; // obj1 background
		        init.at(5)= "T"; // obj2 background
		      }
		      break;
		    case 7: /// release tool
		      init.at(0)= "T"; // hand obj1
		      init.at(1)= "A"; // hand obj2
		      init.at(2)= "N"; // hand background
		      /// the last three relations should be initialized by Vision system.
		      if(is_initial_fixed)
		      {
		        init.at(3)= "A"; // obj1 obj2
		        init.at(4)= "N"; // obj1 background
		        init.at(5)= "A"; // obj2 background
		      }
		      break;
		    case 8: /// unload
		    case 18: /// pour
		      init.at(0)= "N"; // hand obj1
		      init.at(1)= "N"; // hand obj2
		      init.at(2)= "N"; // hand background
		      /// the last three relations should be initialized by Vision system.
		      if(is_initial_fixed)
		      {
		        init.at(3)= "N"; // obj1 obj2
		        init.at(4)= "T"; // obj1 background
		        init.at(5)= "T"; // obj2 background
		      }
		    break;
		    case 15: /// push apart by grasp
		    case 16: /// push apart by holding  
		      init.at(0)= "N"; // hand obj1
		      init.at(1)= "N"; // hand obj2
		      init.at(2)= "N"; // hand background
		      /// the last three relations should be initialized by Vision system.
		      if(is_initial_fixed)
		      {
		        init.at(3)= "T"; // obj1 obj2
		        init.at(4)= "T"; // obj1 background
		        init.at(5)= "T"; // obj2 background
		      }
		      
		      break;
		  }
		  
		  actual_relations = init;
		  actual_relations_int_msgs.data.clear();
		  assert(actual_relations_int_msgs.data.size()==0);
		  for(int i=0; i<(int)actual_relations.size(); i++)
		  {
		    if(actual_relations.at(i)=="T")
		      actual_relations_int_msgs.data.push_back(1);
		    else if(actual_relations.at(i)=="N")
		      actual_relations_int_msgs.data.push_back(0);
		    else if(actual_relations.at(i)=="A")
		      actual_relations_int_msgs.data.push_back(-1);
		  }
		  actual_relations_last = actual_relations;
		  relations_outport.write(actual_relations);
		  actual_relations_int_msgs_outport.write(actual_relations_int_msgs);
		  log(Warning)<<"perception::initialize_relations >> "<<relations_outport.getName()<<" sent the actual_relations"<<endlog();
		  log(Warning)<<"perception::initialize_relations >> init= "<<init.at(3)<<" "<<init.at(4)<<" "<<init.at(5)<<endlog();
		  log(Warning)<<"perception::initialize_relations >> actual_relations= "<<actual_relations.at(3)<<" "<<actual_relations.at(4)
		    <<" "<<actual_relations.at(5)<<endlog();
		}
		void Perception::save_relations()
		{
		  actual_relations_last = actual_relations;
// 		  for(int i=0; i<(int)actual_relations_last.size();i++)
// 		  {
// 		    //log(Warning)<<"perception::save_relations >> Saved relation "<<i<<"  is= "<<actual_relations_last.at(i)<<endlog();
// 		  }
		}
		void Perception::load_relations()
		{
		  actual_relations = actual_relations_last;
		  convert_relation_int();
		  actual_relations_int_msgs_outport.write(actual_relations_int_msgs);
		  relations_outport.write(actual_relations);
		  for(int i=0; i<(int)actual_relations.size();i++)
		  {
		    log(Warning)<<"perception::load_relations >> Loaded relation "<<i<<"  is= "<<actual_relations.at(i)<<endlog();
		  }
		}
		void Perception::update_relation(int relation_index, std::string relation_value)
		{
		  if(enable_update_relation)
		  {
		    if(relation_index>= (int)relation_properties.size())
		    {
		      log(Error)<<"Perception::update_relation >> requested relation_index is out of range"<<endlog();
		    }
		    else if(relation_properties.at(relation_index).type==RELATION_DONTCARE)
		    {
		      ////log(Warning)<<"Perception::update_relation >> requested relation is marked as DONT-CARE, no update"<<endlog();
		    }
		    
		    else if(relation_index<(int)actual_relations.size())
		    {
		      if (actual_relations.at(relation_index)!= relation_value)
		      { 
			//log(Warning)<<"Perception::update_relation >> Relation "<<relation_index<<" is updated to "<<relation_value<<endlog();
			actual_relations.at(relation_index) = relation_value;
			convert_relation_int();
			actual_relations_int_msgs_outport.write(actual_relations_int_msgs);
			relations_outport.write(actual_relations);
			log(Warning)<<"Perception::update_relation >> "<<relations_outport.getName()<<" sent the actual_relations"<<endlog();
			//compare_relations();
		      }
		      else
		      {
			//log(Warning)<<"Perception::update_relation >> Relation "<<relation_index<<" is ALREADY equal to "<<relation_value<<endlog();
		      }
		    }
		    else // out of range
		      log(Error)<<"Perception::update_relation >> The requested index= "<<relation_index
			<<" is bigger than size of actual_relations= "<<actual_relations.size()<<endlog();
		  }
		}
		void Perception::calculate_current_arm_goal()
		{
		  
		  //log(Warning)<<"Perception::calculate_current_arm_goal >> starts"<<endlog();
		  TaskContext* coordinator_task;
		  coordinator_task = this->getPeer("coordinator");
		  TaskContext* action_task;
		  action_task = this->getPeer("action");
// 		  int current_goal_point_int;
// 		  ROBOT_PRIMITIVE_ARGS current_goal_point= NO_ARG;
		  if(action_task==NULL)
		    log(Error)<<"perception::calculate_current_arm_goal >> 'action' task is not defined"<<endlog();
		  else if(coordinator_task==NULL)
		    log(Error)<<"coordinator task is not defined"<<endlog();
		  else
		  {
		    /// get property from action task
		    Property< std::vector<int> > arm_goal_points_property = action_task->getProperty("armGoalPoints");
		    std::vector<int> arm_goal_points = arm_goal_points_property.get();
		    /// get property from coordinator task
		    Property< int > move_index_property = coordinator_task->getProperty("moveIndex");
		    int move_index = move_index_property.get();
		    /// get the arm_goal_point at current state_index
		    move_index--;
		    //log(Warning)<<"perception::calculate_current_arm_goal >> move_index = "<<move_index<<endlog();
		    if(move_index==-1)
		    {
		      current_goal_point_int=7;
		      current_goal_point = ERROR;
		    }
		    else
		    {
		      current_goal_point_int = arm_goal_points.at(move_index);
		      current_goal_point = static_cast<ROBOT_PRIMITIVE_ARGS>(arm_goal_points.at(move_index));
		    }
		  }
		  
		    //arm_status_string = "arm_at_"+ current_goal_point;
		}
		void Perception::set_current_arm_goal(int goal_index)
		{
		  current_goal_point = static_cast<ROBOT_PRIMITIVE_ARGS>(goal_index);
		}
		void Perception::set_desired_pose (int point_index)
		{
// 		  log(Error)<<"perception::set_desired_pose >> start with point_index= "<<point_index<<endlog();
		  ROBOT_PRIMITIVE_ARGS point_index_enum = static_cast<ROBOT_PRIMITIVE_ARGS> (point_index);
		  geometry_msgs::Pose desired_pose_tool_table,desired_pose_tool_robot,desired_pose_arm_table;
		  // write to the port
		  std_msgs::Int16 arm_goal_index;
		  arm_goal_index.data = (short) point_index;
		  arm_goal_msgs_outport.write(arm_goal_index);
		  switch(point_index_enum)
		  {
		    case OBJ1:
		      log(Error)<<"perception::set_desired_pose >> The desired pose is obj1=\n"<<obj1_table<<endlog();
		      desired_pose_tool_table = obj1_table;
		      is_desired_pose_set = true;
		      break;
		    case OBJ2:
		      //log(Warning)<<"perception::set_desired_pose >> The desired pose is obj2 "<<endlog();
		      desired_pose_tool_table = obj2_table;
		      is_desired_pose_set = true;
		      break;
		    case HOME:
		      //log(Warning)<<"perception::set_desired_pose >> The desired pose is home "<<endlog();
		      desired_pose_tool_table = home_table;
		      is_desired_pose_set = true;
		      break;
		    case HOME2:
		      //log(Warning)<<"perception::set_desired_pose >> The desired pose is home2 "<<endlog();
		      desired_pose_tool_table = home2_table;
		      is_desired_pose_set = true;
		      break;
		    case GOAL:
		      /// for action which need holding from top, modify the goal according to current tool pose.
		      if(action_type==14 || action_type==16 || action_type==17)
			goal_table.position.z = tool_actual_table.position.z - 0.02;
		      desired_pose_tool_table = goal_table;
		      //log(Error)<<"perception::set_desired_pose >> The desired xxx pose is goal\n "<<goal_table<<endlog();
		      is_desired_pose_set = true;
		      break;
		    case GOAL2:
		      //log(Warning)<<"perception::set_desired_pose >> The desired pose is goal2 "<<endlog();
		      desired_pose_tool_table = goal_table2;
		      is_desired_pose_set = true;
		      break;
		    case GOAL3:
		      //log(Warning)<<"perception::set_desired_pose >> The desired pose is goal3 "<<endlog();
		      desired_pose_tool_table = goal_table3;
		      is_desired_pose_set = true;
		      break;
		    case BACKGROUND:
		      //log(Warning)<<"perception::set_desired_pose >> The desired pose is background "<<endlog();
		      desired_pose_tool_table = background_table;
		      is_desired_pose_set = true;
		      break;
		    case ERROR:
		      log(Error)<<"perception::set_desired_pose >> The Error pose is the current position plus an offset in Z direction"<<endlog();
		      double norm_quat;
		      norm_quat = tool_actual_table.orientation.x*tool_actual_table.orientation.x +
		      tool_actual_table.orientation.y*tool_actual_table.orientation.y + tool_actual_table.orientation.z*tool_actual_table.orientation.z +
		      tool_actual_table.orientation.w*tool_actual_table.orientation.w;
		      desired_pose_tool_table = tool_actual_table;
		      norm_quat = sqrt(norm_quat);
		      if (norm_quat<0.9)
		      {
			log(Error)<<"perception::set_desired_pose >> orientation is not correct. norm_quat= "<<norm_quat<<endlog();
			desired_pose_tool_table = tool_actual_table;
			desired_pose_tool_table.orientation.x = 1;
			desired_pose_tool_table.orientation.y = 0;
			desired_pose_tool_table.orientation.z = 0;
			desired_pose_tool_table.orientation.w = 0;
		      }
		      else
			desired_pose_tool_table = tool_actual_table;
		      log(Error)<<"perception::set_desired_pose >> The current pose is="<<tool_actual_table<<endlog();
		      log(Error)<<"perception::set_desired_pose >> The erro pose is="<<desired_pose_tool_table<<endlog();
		      desired_pose_tool_table.position.z += 0.15;
		      log(Error)<<"perception::set_desired_pose >> The erro pose is="<<desired_pose_tool_table<<endlog();
		      is_desired_pose_set = true;
		      break;
		    default:
		      //log(Error)<<"perception::set_desired_pose >> the point index is not valid point_index= "<<point_index<<"    p= "<<point_index_enum<<endlog();
		      is_desired_pose_set = false;
		      break;
		  }
		  log(Error)<<"perception::set_desired_pose >> The desired tool pose in table frame is=\n"<<desired_pose_tool_table;
		  std::vector<double> euler_angles = getZYXEulerfromPose(desired_pose_tool_table);
		  //log(Warning)<<" perception::set_desired_pose >> Euler angles are=\t"<<euler_angles.at(0)<<"\t"<<euler_angles.at(1)<<"\t"<<euler_angles.at(2)<<endlog();
		  desired_pose_outport.write(desired_pose_tool_table);
		  adjust_tool(desired_pose_tool_table,desired_pose_arm_table,tool_offset,-1);
		  desired_pose_arm_outport.write(desired_pose_arm_table);
		  tool_desired_table = desired_pose_tool_table;
		  calculate_move_direction();
		  transform_table_to_robot(desired_pose_tool_table,desired_pose_tool_robot);
		  adjust_tool(desired_pose_tool_robot,arm_desired_robot,tool_offset,-1);
		  //log(Warning)<<"perception::set_desired_pose >> The tool_offset is=\n"<<tool_offset.data[0]<<" "<<
// 		  tool_offset.data[1]<<" "<<tool_offset.data[2]<<endlog();
		  //log(Warning)<<"perception::set_desired_pose >> The desired arm pose in robot frame is=\n"<<arm_desired_robot;
		  desired_pose_arm_robot_outport.write(arm_desired_robot);
		}
		void Perception::set_arm_status()
		{
		  switch(current_goal_point)
		  {
		    case OBJ1:
		      arm_status = ARM_AT_OBJ1;
		      
		      break;
		    case OBJ2:
		      arm_status = ARM_AT_OBJ2;
		      break;
		    case HOME:
		      arm_status = ARM_AT_HOME;
		      break;
		    case HOME2:
		      arm_status = ARM_AT_HOME2;
		      break;
		    case GOAL:
		      arm_status = ARM_AT_GOAL;
		      break;
		    case GOAL2:
		      arm_status = ARM_AT_GOAL2;
		      break;
		    case BACKGROUND:
		      arm_status = ARM_AT_BACKGROUND;
		      break;
		    case ERROR:
		      arm_status = ARM_AT_ERROR;
		      break;
		    case GOAL3:
		      arm_status = ARM_AT_GOAL3;
		      break;
		    default:
		      log(Error)<<"perception::set_arm_status >> the current_goal_point is not valid= "<<current_goal_point<<endlog();
		      break;
		  }
		  //log(Warning)<<"perception::set_arm_status>> The arm_status= "<<arm_status<<endlog();
		}
		float Perception::calculate_time(float desired_velocity)
		{
		  float x_des,y_des,z_des,x_act,y_act,z_act;
		  x_act = arm_actual_robot.position.x;
		  y_act = arm_actual_robot.position.y;
		  z_act = arm_actual_robot.position.z;
		  x_des = arm_desired_robot.position.x;
		  y_des = arm_desired_robot.position.y;
		  z_des = arm_desired_robot.position.z;
		  //log(Warning)<<"perception::calculate_time >> the desired pose is= "<<x_des<<" "<<y_des<<" "<<z_des<<endlog();
		  //log(Warning)<<"perception::calculate_time >> the actual pose is= "<<x_act<<" "<<y_act<<" "<<z_act<<endlog();
		  float distance = sqrt((x_des-x_act)*(x_des-x_act)+(y_des-y_act)*(y_des-y_act)+(z_des-z_act)*(z_des-z_act));
		  //log(Warning)<<"perception::calculate_time >> the distance from current pose to desired is= "<<distance<<endlog();
		  //float desired_velocity = 0.1;
		  float desired_time = distance / desired_velocity;
		  /// calculate the desired time for change in the orientation
		  KDL::Frame kdl_desired, kdl_actual;
		  tf::poseMsgToKDL(arm_actual_robot,kdl_actual);
		  tf::poseMsgToKDL(arm_desired_robot,kdl_desired);
		  double a_desired,b_desired,c_desired;
		  kdl_desired.M.GetEulerZYX(a_desired,b_desired,c_desired);
		  a_desired = 180* a_desired / M_PI;
		  b_desired = 180* b_desired / M_PI;
		  c_desired = 180* c_desired / M_PI;
		  //log(Warning)<<"perception::calculate_time >> the desired angles are= "<<a_desired<<" "<<b_desired<<" "<<c_desired<<endlog();
		  double a_actual,b_actual,c_actual;
		  kdl_actual.M.GetEulerZYX(a_actual,b_actual,c_actual);
		  a_actual = 180* a_actual / M_PI;
		  b_actual = 180* b_actual / M_PI;
		  c_actual = 180* c_actual / M_PI;
		  //log(Warning)<<"perception::calculate_time >> the actual angles are= "<<a_actual<<" "<<b_actual<<" "<<c_actual<<endlog();
		  /// new method for angular distance
// 		  KDL::Rotation R_before,R_after,R_diff;
// 		  R_before = kdl_actual.M;
// 		  R_after = kdl_desired.M;
// 		  R_diff = R_after*R_before.Inverse();
// 		  double a_diff, b_diff, c_diff;
// 		  R_diff.GetEulerZYX(a_diff,b_diff,c_diff);
// 		  a_diff = 180* a_diff / M_PI;
// 		  b_diff = 180* b_diff / M_PI;
// 		  c_diff = 180* c_diff / M_PI;
// 		  //log(Warning)<<"perception::calculate_time >> the diff angles are= "<<a_diff<<" "<<b_diff<<" "<<c_diff<<endlog();
		  /// angular_distance in degrees
		  float angular_distance = sqrt((a_desired-a_actual)*(a_desired-a_actual)+(b_desired-b_actual)*(b_desired-b_actual)+(c_desired-c_actual)*(c_desired-c_actual));
		  //log(Warning)<<"perception::calculate_time >> the angular distance is = "<<angular_distance<<endlog();
		  float desired_velocity_angular = 25; // 5 degrees per second
		  float angular_time = angular_distance / desired_velocity_angular;
		  //log(Warning)<<"perception::calculate_time >> calculated angular time is = "<<angular_time<<endlog();
		  if(angular_time>desired_time)
		  {
		    //log(Warning)<<"perception::calculate_time >> the desired time is (angular)= "<<angular_time<<endlog();
		    return angular_time;
		  }
		  //log(Warning)<<"perception::calculate_time >> the desired time is= "<<desired_time<<endlog();
		  return desired_time;
		  
		}
		bool Perception::is_arm_above_desired()
		{
		  bool retval = false;
		  float xy_threshold =0.03;
		  float x_des,y_des,x_act,y_act;
		  x_act = tool_actual_table.position.x;
		  y_act = tool_actual_table.position.y;
		  x_des = tool_desired_table.position.x;
		  y_des = tool_desired_table.position.y;
		  ////log(Warning)<<"perception::calculate_time >> the desired pose is= "<<x_des<<" "<<y_des<<" "<<z_des<<endlog();
		  ////log(Warning)<<"perception::calculate_time >> the actual pose is= "<<x_act<<" "<<y_act<<" "<<z_act<<endlog();
		  float distance = sqrt((x_des-x_act)*(x_des-x_act)+(y_des-y_act)*(y_des-y_act));
		  //log(Warning)<<"perception::is_arm_above_desired >> the distance in XY plane= "<<distance<<"  threshold is= "<<xy_threshold<<endlog();
		  //float desired_velocity = 0.1;
		  //float desired_time = distance / desired_velocity;
		  if(distance<xy_threshold) retval=true;
		  //log(Warning)<<"perception::is_arm_above_desired >> is arm above the desired point?  "<<retval<<endlog();
		  return retval;
		  
		}
		void Perception::adjust_tool(geometry_msgs::Pose pose_before, geometry_msgs::Pose& pose_after, KDL::Vector tool_offset,int direction)
		{
		  /// if direction== -1 , calculates arm pose from TCP pose
		  /// if direction== 1 ,  calculates TCP pose from arm pose
// 		  tool_offset.position.x *= direction;
// 		  tool_offset.position.y *= direction;
// 		  tool_offset.position.z *= direction;
		  
		  KDL::Frame pose_before_kdl, pose_after_kdl, tool_offset_kdl;
		  tf::poseMsgToKDL(pose_before,pose_before_kdl);
// 		  //log(Warning)<<"perception::adjust_tool >> before orientation ="<<pose_before.orientation<<endlog();
		  double x,y,z,w;
		  pose_before_kdl.M.GetQuaternion(x,y,z,w);
// 		  //log(Warning)<<"perception::adjust_tool >> KDL before orientation ="<<x<<" "<<y<<" "<<z<<" "<<w<<endlog();
		  double a,b,c;
		  pose_before_kdl.M.GetEulerZYX(a,b,c);
// 		  //log(Warning)<<"perception::adjust_tool >> KDL before euler angles= "<<a*180/3.14<<" "<<b*180/3.14<<" "<<c*180/3.14<<endlog();
		  KDL::Rotation rot_before_kdl = pose_before_kdl.M;
		  KDL::Vector adjust_vector = rot_before_kdl*tool_offset;
// 		  log(Warning)<<"perception::adjust_tool >> applied tool offset= "<<tool_offset.data[0]<<" "
// 		    <<tool_offset.data[1]<<" "<<tool_offset.data[2]<<" "<<endlog();
		  pose_after_kdl = pose_before_kdl;
		  pose_after_kdl.p += direction*adjust_vector;
		  pose_after_kdl.M.GetQuaternion(x,y,z,w);
// 		  //log(Warning)<<"perception::adjust_tool >> KDL after orientation ="<<x<<" "<<y<<" "<<z<<" "<<w<<endlog();
		  pose_after_kdl.M.GetEulerZYX(a,b,c);
// 		  //log(Warning)<<"perception::adjust_tool >> KDL after euler angles= "<<a*180/3.14<<" "<<b*180/3.14<<" "<<c*180/3.14<<endlog();
		  tf::poseKDLToMsg(pose_after_kdl,pose_after);
		  pose_after.orientation = pose_before.orientation;
// 		  //log(Warning)<<"perception::adjust_tool >> after orientation ="<<pose_after.orientation<<endlog();
		}
		float Perception::distance_xy(geometry_msgs::Pose pose1,geometry_msgs::Pose pose2 )
		{
		  float retval;
		  retval = sqrt((pose1.position.x-pose2.position.x)*(pose1.position.x-pose2.position.x)+
			    (pose1.position.y-pose2.position.y)*(pose1.position.y-pose2.position.y));
		  return retval;
		}
		void Perception::print_arm_pose()
		{
// 		  //log(Warning)<<"============================================================"<<endlog();
		  //log(Warning)<<"perception::print_arm_pose >> Printing arm and tool poses\n"<<endlog();
		  //log(Warning)<<"perception::print_arm_pose >> Actual tool pose in table frame=\n"<< tool_actual_table<<endlog();
		  //log(Warning)<<"perception::print_arm_pose >> Actual arm pose in table frame=\n"<< arm_actual_table<<endlog();
		  //log(Warning)<<"perception::print_arm_pose >> Actual arm pose in robot frame=\n"<< arm_actual_robot<<endlog();
		  
// 		  //log(Warning)<<"============================================================"<<endlog();
		}
		void Perception::set_orientation(geometry_msgs::Pose &pose,float a_degree_table, float b_degree_table, float c_degree_table)
		{
		  float a_radian_robot = a_degree_table*M_PI/180; 
		  float b_radian_robot = b_degree_table*M_PI/180;
		  float c_radian_robot = c_degree_table*M_PI/180;
		  KDL::Rotation rot1= KDL::Rotation::EulerZYX(a_radian_robot,b_radian_robot,c_radian_robot);
		  KDL::Frame frame1;
		  frame1.M = rot1;
		  geometry_msgs::Pose pos1;
		  tf::poseKDLToMsg(frame1,pos1);
		  pose.orientation = pos1.orientation;
		  //log(Warning)<<"perception::set_orientation >> output pose is="<<pose<<endlog();
		}
		
		void Perception::transform_table_to_robot(geometry_msgs::Pose& pose_table, geometry_msgs::Pose& pose_robot)
		{
		  // this function converts from table frame to robot frame
		  		  
		  KDL::Frame pose_robot_kdl,pose_table_kdl;
		  tf::poseMsgToKDL(pose_table,pose_table_kdl);
		  //tf::transformMsgToTF(pose_table,pose_table_kdl);
		  pose_robot_kdl = robot_table_transform * pose_table_kdl;
		  
		  tf::poseKDLToMsg(pose_robot_kdl,pose_robot);
		  ////log(Warning)<<"perception::transform_table_to_robot >> pose_table= "<<pose_table;
		  ////log(Warning)<<"perception::transform_table_to_robot >> pose_robot= "<<pose_robot;
		}
		void Perception::transform_robot_to_table(geometry_msgs::Pose& pose_robot, geometry_msgs::Pose& pose_table)
		{
		  // this function converts from robot frame to table frame
		  		  
		  KDL::Frame pose_robot_kdl,pose_table_kdl;
		  tf::poseMsgToKDL(pose_robot,pose_robot_kdl);
		  //tf::transformMsgToTF(pose_table,pose_table_kdl);
		  pose_table_kdl = table_robot_transform * pose_robot_kdl;
		  tf::poseKDLToMsg(pose_table_kdl,pose_table);
		  ////log(Warning)<<"perception::transform_robot_to_table >> pose_robot= "<<pose_robot;
		  ////log(Warning)<<"perception::transform_robot_to_table >> pose_table= "<<pose_table;
		  
		}
		geometry_msgs::Pose Perception::final_pose()
		{
		  // returns the final goal pose at the end of action
		  // this pose is a point above the current pose.
		  // this makes the arm to move up at the end of action
		  geometry_msgs::Pose final_pose_table,final_pose_robot;
		  //log(Warning)<<"perception::final_pose >> actual arm pose in table frame\n"<<arm_actual_table<<endlog();
		  final_pose_table = arm_actual_table;
		  final_pose_table.position.z += 0.15;
		  if(final_pose_table.position.z > 0.6)
		    final_pose_table.position.z = 0.6;
		  if(action_type!=18 && final_pose_table.position.y < 0.35)
		    final_pose_table.position.y = 0.35;
		  /// after unload, move also in x direction (for salad scenraio)
		  if(action_type== 8)
		  {
		    final_pose_table.position.z += 0.05;
		    final_pose_table.position.y -= 0.05;
// 		    final_pose_table.position.x = -0.37;
// 		    final_pose_table.position.y = 0.52;
// 		    final_pose_table.position.z = 0.21+ hand_offset.data[2];
		    
		  }
		  /// for all actions except cutting, use an upright pose as the final.
		  
		  if(action_type!=10 && action_type!=18 && action_type!=19) 
		  {
		    final_pose_table.orientation.x = -0.70710678118;
		    final_pose_table.orientation.y = 0.70710678118;
		    final_pose_table.orientation.z = 0;
		    final_pose_table.orientation.w = 0;
		  }
		  /// fixed position (arm pose in table frame)
// 		  /*/*if(arm_number==0) /// right arm
// 		  {
// 		    final_pose_table.position.x = 0.350;
// 		    final_pose_table.position.y = 0.530;
// 		    final_pose_table.position.z = 0.460;
// 		    final_pose_table.orientation.x = -0.70710678118;
// 		    final_pose_table.orientation.y = 0.70710678118;
// 		    final_pose_table.orientation.z = 0;
// 		    final_pose_table.orientation.w = 0;
// 		  }
// 		  else if(arm_number==1) /// leftt arm
// 		  {
// 		    final_pose_table.position.x = -0.350;
// 		    final_pose_table.position.y = 0.530;
// 		    final_pose_table.position.z = 0.460;
// 		    final_pose_table.orientation.x = -0.70710678118;
// 		    final_pose_table.orientation.y = 0.70710678118;
// 		    final_pose_table.orientation.z = 0;
// 		    final_pose_table.orientation.w = 0;
// 		  }*/*/
		  // use a known fixed orientation
// 		  final_pose_table.orientation.x = 0.496104;
// 		  final_pose_table.orientation.y = 0.868201;
// 		  final_pose_table.orientation.z = -0.00332606;
// 		  final_pose_table.orientation.w = 0.00982785;
		  //log(Warning)<<"perception::final_pose >> transform from table to robot frame"<<endlog();
		  transform_table_to_robot(final_pose_table,final_pose_robot);
		  // use the orientation from actual arm pose in robot frame
		  final_pose_robot.orientation = arm_actual_robot.orientation;
		  //log(Warning)<<"perception::final_pose >> final arm pose in table frame\n"<<final_pose_table<<endlog();
		  //log(Warning)<<"perception::final_pose >> final arm pose in robot frame\n"<<final_pose_robot<<endlog();
		  return final_pose_robot;
		}
		void Perception::calculate_saw_motion()
		{
		  //log(Warning)<<"perception::calculate_saw_motion >> starts ..."<<endlog();
		  float saw_length = -0.015;
		  //log(Warning)<<"perception::calculate_saw_motion >> knife_angle_cosine= "<<knife_angle_cosine<<endlog();
		      //log(Warning)<<"perception::calculate_saw_motion >> knife_angle_sine= "<<knife_angle_sine<<endlog();
		  float saw_x = saw_length * knife_angle_cosine;
		  float saw_y = saw_length * knife_angle_sine;
		  KDL::Vector saw_vector_table;
		  saw_vector_table.data[0] = saw_x;
		  saw_vector_table.data[1] = saw_y;
		  saw_vector_table.data[2] = 0;
		  //log(Warning)<<"perception::calculate_saw_motion >> saw vector in table frame= "<<saw_vector_table.data[0]
// 			      <<" "<<saw_vector_table.data[1] <<" "<<saw_vector_table.data[2] <<endlog();
		  double a,b,c;
		  robot_table_transform.M.GetEulerZYX(a,b,c);
		  //log(Warning)<<"perception::calculate_saw_motion >> robot_table_transform angles"<<a*180/M_PI<<" "<<b*180/M_PI<<" "<<c*180/M_PI<<endlog();
		  saw_vector_robot = robot_table_transform.M * saw_vector_table;
		  //log(Warning)<<"perception::calculate_saw_motion >> saw vector in table frame= "<<saw_vector_robot.data[0]
// 			      <<" "<<saw_vector_robot.data[1] <<" "<<saw_vector_robot.data[2] <<endlog();
		}
		void Perception::calculate_stir_motion()
		{
		  //log(Warning)<<"perception::calculate_stir_motion >> starts ..."<<endlog();
		  float stir_length = -0.05;
		  float stir_x = stir_length; 
		  float stir_y = stir_length;//saw_length * knife_angle_sine;
		  KDL::Vector stir_vector_table;
		  stir_vector_table.data[0] = stir_x;
		  stir_vector_table.data[1] = stir_y;
		  stir_vector_table.data[2] = 0;
		  stir_vector_robot = robot_table_transform.M * stir_vector_table;
		  
		}
		std::vector<double> Perception::getZYXEulerfromPose(geometry_msgs::Pose & pose)
		{
		  KDL::Frame frame1;
		  tf::poseMsgToKDL(pose,frame1);
		  double angle_alfa,angle_beta,angle_gamma;
		  frame1.M.GetEulerZYX(angle_alfa,angle_beta,angle_gamma);
		  std::vector<double> retval;
		  retval.push_back(angle_alfa*180/PI);
		  retval.push_back(angle_beta*180/PI);
		  retval.push_back(angle_gamma*180/PI);
		  return retval;
		}
		void Perception::set_tool_offset(int tool_type)
		{
		  hand_offset.data[0] = 0;
		  hand_offset.data[1] = 0;
		  hand_offset.data[2] = 0.22;
// 		  
// 		  /// yellow knife
		  knife_offset.data[0] = 0;
		  knife_offset.data[1] = 0.035;
		  knife_offset.data[2] = 0.28;
// 		  /// bread cutter
// // 		  knife_offset.data[0] = 0;
// // 		  knife_offset.data[1] = 0.015;
// // 		  knife_offset.data[2] = 0.28;
// 		  
		  spoon_offset.data[0] = 0;
		  spoon_offset.data[1] = 0;
		  spoon_offset.data[2] = hand_offset.data[2]+ 0.15;
// 		  
		  cleaver_offset.data[0] = 0;
		  cleaver_offset.data[1] = 0.09;
		  cleaver_offset.data[2] = 0.35;
		  switch(tool_type)
		  {
		    case TOOL_HAND:
		    {
		      tool_offset = hand_offset;
		      log(Warning)<<"perception::set_tool_offset >> tool is TOOL_HAND";
		      //log(Warning)<<"\t"<<tool_offset.data[0]<<" "<<tool_offset.data[1]<<" "<<tool_offset.data[2]<<endlog();
		      break;
		    }
		    case TOOL_KNIFE:
		    {
		      tool_offset = knife_offset;
		      //log(Warning)<<"perception::set_tool_offset >> tool is TOOL_KNIFE";
		      //log(Warning)<<"\t"<<tool_offset.data[0]<<" "<<tool_offset.data[1]<<" "<<tool_offset.data[2]<<endlog();
		      break;
		    }
		    case TOOL_SPOON:
		    {
		      tool_offset = spoon_offset;
		      //log(Warning)<<"perception::set_tool_offset >> tool is TOOL_SPOON";
		      //log(Warning)<<"\t"<<tool_offset.data[0]<<" "<<tool_offset.data[1]<<" "<<tool_offset.data[2]<<endlog();
		      break;
		    }
		    case TOOL_CLEAVER:
		    {
		      tool_offset = cleaver_offset;
		      break;
		    }
		    default:
		    {
		    }
		  }
		}
		void Perception::calculate_move_direction()
		{
		  movement_direction.data[0]= tool_desired_table.position.x - tool_actual_table.position.x;
		  movement_direction.data[1]= tool_desired_table.position.y - tool_actual_table.position.y;
		  movement_direction.data[2]= tool_desired_table.position.z - tool_actual_table.position.z;
		  movement_direction.Normalize();
		  //log(Warning)<<"perception::calculate_move_direction >> move direction is="<<movement_direction.Normalize();
		}
		void Perception::convert_relation_int()
		{
		  assert(actual_relations.size()==actual_relations_int_msgs.data.size());
		  for(int i=0; i<(int)actual_relations.size(); i++)
		  {
		    if(actual_relations.at(i)=="T")
		      actual_relations_int_msgs.data.at(i)=1;
		    else if(actual_relations.at(i)=="N")
		      actual_relations_int_msgs.data.at(i)=0;
		    else if(actual_relations.at(i)=="A")
		      actual_relations_int_msgs.data.at(i)=-1;
		  }
		}
		void Perception::calculate_goal_push_apart(float &X_goal, float &Y_goal, const float R_distance)
		{
		  /// get the x and y coordinates of obj1 and obj2
		  float x1 = obj1_table.position.x;
		  float y1 = obj1_table.position.y;
		  float x2 = obj2_table.position.x;
		  float y2 = obj2_table.position.y;
		  /// line L: line that passes through obj1 and is perpendicular to the line passing obj1 and obj2
		  /// y= L_slope*x + L_intercept
		  /// calculate L_slope and L_intercept
		  float L_slope,L_intercept;
		  L_slope = (x1-x2)/(y2-y1);
		  L_intercept = y1-L_slope*x1;
		  
		  /// find the goal point (goal_x,goal_y), which is a point on line L with distance R_distance with obj1 in XY plane.
		  /// two equations must be solved:
		  /// 1: Y=L_slope*X + L_intercept
		  /// 2: (Y-y1)^2 + (X-x1)^2 = R_distance^2
		  /// by replacing 1 in 2 we get : AX^2+ BX + C=0 , where A,B and C are as follows:
		  float A = 1+pow(L_slope,2);
		  float B = 2*(L_slope*L_intercept-L_slope*y1-x1);
		  float C = pow(x1,2)+ pow((L_intercept-y1),2) - pow(R_distance,2);
		  /// solving for X we get two solutions: X1 and X2
		  float X1 = (-1*B + sqrt(pow(B,2)-4*A*C) )/ (2*A);
		  float X2 = (-1*B - sqrt(pow(B,2)-4*A*C) )/ (2*A);
		  /// using 1, we get the corresponding Y1 and Y2
		  float Y1= L_slope * X1 + L_intercept;
		  float Y2= L_slope * X2 + L_intercept;
		  
		  /// select between pair (X1,Y1) and (X2,Y2), whichever that is closer to the origin.
		  float dist1 = X1*X1 + Y1*Y1;
		  float dist2 = X2*X2 + Y2*Y2;
		  if(dist1<dist2)
		  {
		    X_goal = X1;
		    Y_goal = Y1;
		  }
		  else
		  {
		    X_goal = X2;
		    Y_goal = Y2;
		  }
		  
		}
		void Perception::set_cutting_point(int cutting_index)
		{
		  //log(Warning)<<"perception::set_cutting_point >> setting the cutting point. Cutting index= "<<cutting_index<<endlog();
		  float disp_y = 0;
		  float distance_y_abs = 0.03;
		  if(cutting_index==2)
		  {
		    disp_y = -1*distance_y_abs;
		    home2_table.position.z = 0.1;
		  }
		  else if(cutting_index==1)
		  {
		    disp_y = -1*distance_y_abs;
		    home2_table.position.z = 0.1;
		  }
		  else if(cutting_index==0)
		  {
		    //disp_y = distance_y_abs;
		  }
		  else 
		  {
		    disp_y = 0;
		  }
		  obj2_table.position.y += disp_y;
		  home2_table.position.y = obj2_table.position.y;
		}
		void Perception::set_cutting_sidepoint()
		{
		  //log(Warning)<<"perception::set_cutting_sidepoint >> setting the cutting side pose. Current pose="<<tool_actual_table<<endlog();
		  float disp_y = 0.015;
		  goal_table3 = tool_actual_table;
		  goal_table3.position.y += disp_y;
		  //log(Warning)<<"perception::set_cutting_sidepoint >> setting the cutting side pose. side pose="<<goal_table3<<endlog();
		}
		void Perception::detect_tool_background_relation()
		{
		  //log(Warning)<<"perception::detect_tool_background_relation >> Touching is confirmed"<<endlog();
		  update_relation(4,"T");
// 		  //log(Warning)<<"perception::detect_tool_background_relation >> starts"<<endlog();
// 		  if(tool_actual_table.position.z<0.001)
// 		  {
// 		    //log(Warning)<<"perception::detect_tool_background_relation >> Z is less than a threshold, Touching is confirmed";
// 		    update_relation(4,"T");
// 		  }
		}
		void Perception::move_main2()
		{
// 		  obj1_table.position.x -= 0.15;
// 		  obj1_table.position.y += 0.1;
		  obj2_table.position.x += 0.25;
		  obj2_table.position.y += 0.1;
		  goal_table2.position.x = obj2_table.position.x;
		  goal_table2.position.y = obj2_table.position.y;
		}
		void Perception::move_main3()
		{
		  obj1_table.position.z -= 0.03;
// 		  obj1_table.position.y += 0.1;
		  
		}



}


ORO_CREATE_COMPONENT( UseCase::Perception )
