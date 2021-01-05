#include "romeo_grasper/modeledobject.h"

#include <ros/console.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

ModeledObject::ModeledObject(ros::NodeHandle *nh_, moveit_visual_tools::MoveItVisualToolsPtr &visual_tools, const std::string model_name, const std::string rviz_name, const geometry_msgs::PoseStamped pose, const bool is_reference_object, const bool camera_positioned, const bool verbose)
{    
    verbose_ = verbose;
    bool debug;
    nh_->param("debug", debug, false);
    if(debug)
    {
        if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
            ros::console::notifyLoggerLevelsChanged();
        }
        // If debug is on the verbose info should too
        verbose_ = true;
    }

    is_reference_object_ = is_reference_object;
    camera_positioned_ = camera_positioned;

    if(verbose_)
        ROS_INFO_STREAM("Creating ModeledObject with name: " << rviz_name << " from model: " << model_name);

    std::string default_value = "/home/lluis/catkin_ws/src/romeo_grasper/data/models/";
    nh_->param("models_directory", models_directory_,default_value);
    std::string::iterator it = models_directory_.end() - 1;
    if (*it != '/')
    {
        models_directory_.append("/");
    }

    if(verbose_)
        ROS_INFO_STREAM("Folder of models: " << models_directory_);

    model_name_ = model_name;
    model_folder_ = models_directory_ + model_name_;
    model_filename_ = model_folder_ + "/tracking_model.ao";

    float offset_x, offset_y, offset_z;
    if(!nh_->getParam("models/" + model_name_ + "/offset_x", offset_x) ||
            !nh_->getParam("models/" + model_name_ + "/offset_y", offset_y) ||
            !nh_->getParam("models/" + model_name_ + "/offset_z", offset_z))
    {
        nh_->param("models/" + model_name_ + "/offset_x", offset_x, 0.0f);
        nh_->param("models/" + model_name_ + "/offset_y", offset_y, 0.0f);
        nh_->param("models/" + model_name_ + "/offset_z", offset_z, 0.0f);
        ROS_WARN_STREAM("The config of model: " << model_name << " doesn't exist or some of the offset params are missing");
    }

    offsets.push_back(offset_x);
    offsets.push_back(offset_y);
    offsets.push_back(offset_z);

    default_value = "/object_frame";
    nh_->param("object_frame_id", object_frame_id_, default_value);
    default_value = "/camera_rgb_optical_frame";
    nh_->param("camera_frame_id", camera_frame_id_, default_value);
    default_value = "/base_link";
    nh_->param("base_link_frame_id", base_link_, default_value);

    // Only initialised,
    // till the moment when camera is positioned and the object has been found the pose is useless
    has_object_pose_ = false;
    pose_ajusted_ = false;

    pose_from_camera_ = pose.pose;

    visual_tools_ = visual_tools;
    rviz_name_ = rviz_name;

    // Read MetaBlock features from config file
    int shapeType;
    double size;
    double size_l;

    int shapeTypeDefault = shape_msgs::SolidPrimitive::CYLINDER;
    nh_->param("models/" + model_name_ + "/shapeType", shapeType, shapeTypeDefault);
    nh_->param("models/" + model_name_ + "/size", size, 0.05);
    nh_->param("models/" + model_name_ + "/size_l", size_l, 0.25);
    block_ = new MetaBlock(rviz_name_, pose.header.stamp, pose.pose, shapeType, size, size_l);
    if(verbose_)
        ROS_INFO_STREAM("MetaBlock created with type: " << shapeType << " and sizes: " << size << "/" << size_l);

    // For reference objects we don't need to publish
    if(!is_reference_object_)
    {
        first_update_done_ = false;
        transform_thread_ = boost::shared_ptr<boost::thread>(new boost::thread (boost::bind(&ModeledObject::publishTransforms, this)));
    }
}

ModeledObject::~ModeledObject()
{
    visual_tools_->cleanupCO(rviz_name_);
    visual_tools_->cleanupACO(rviz_name_);
    // TODO: Must I remove transform_thread_ and block_??
    delete block_;
}

void ModeledObject::updatePose(geometry_msgs::PoseStamped pose)
{
    if(!is_reference_object_)
    {
        if(verbose_)
            ROS_INFO_STREAM("Updating pose of ModeledObject: " << rviz_name_);

        pose_from_camera_ = pose.pose;
        if(!first_update_done_)
            first_update_done_ = true;

        pose_ajusted_ = false;

        // Give time to the publisher to make the transformation and publish
        while(!pose_ajusted_)
            sleep(0.5);

        // Transformation from base_link to the camera_link
        tf::StampedTransform transform;
        tf::TransformListener tf_listener;
        try{                      
            ROS_DEBUG_STREAM("Look up for a transform between " << base_link_ << " and " <<  object_frame_id_);
            tf_listener.waitForTransform(base_link_, object_frame_id_,
                                         ros::Time(0), ros::Duration(60.0));
            tf_listener.lookupTransform(base_link_, object_frame_id_,
                                        ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        block_->start_pose = transformToPose(transform);
        ROS_DEBUG_STREAM("Transform from " << base_link_ << " to " << object_frame_id_ << " with offset:\n" << block_->start_pose);

        has_object_pose_ = true;

        // TODO: support more shapeType
        visual_tools_->cleanupCO(rviz_name_);
        visual_tools_->cleanupACO(rviz_name_);

        visual_tools_->publishCollisionCylinder(block_->start_pose, block_->name, block_->size, block_->size_l);
    }else
    {
        pose_from_camera_ = pose.pose;
        block_->start_pose = transformToPose(transformWithOffset());
        has_object_pose_ = true;
    }
}


// Thread to publish constantly the transform from camera_frame to object_frame
void ModeledObject::publishTransforms()
{
    ROS_INFO_STREAM("ModeledObject - Publishing transforms");
    tf::TransformBroadcaster tf_broadcaster;

    ros::Duration sleeper(0.1); // 100ms

    while (ros::ok())
    {
        // TODO: Should I use the time_stamp or the stamp of the pose that I have???
        // time stamp is future dated to be valid for given duration
        ros::Time time_stamp = ros::Time::now() + sleeper;

        if(camera_positioned_ && !is_reference_object_ && first_update_done_)
        {
            if(!pose_ajusted_)
            {
                tr_ajusted_ = transformWithOffset();
                ROS_DEBUG_STREAM("Sending transform child frame: " << object_frame_id_ << " with parent frame: " << camera_frame_id_);
                tf_broadcaster.sendTransform(tf::StampedTransform(tr_ajusted_, time_stamp, camera_frame_id_, object_frame_id_));
                sleeper.sleep(); // need sleep or transform won't publish correctly
                pose_ajusted_ = true;
                if(verbose_)
                    ROS_INFO_STREAM("Frame " << object_frame_id_ << " ajusted according to offsets of model: " << model_name_);
            }else
            {
                ROS_DEBUG_STREAM("Sending transform child frame: " << object_frame_id_ << " with parent frame: " << camera_frame_id_);
                tf_broadcaster.sendTransform(tf::StampedTransform(tr_ajusted_, time_stamp, camera_frame_id_, object_frame_id_));
            }
        }

        sleeper.sleep(); // need sleep or transform won't publish correctly
    }
}

// Returns the transforme of pose with offset in camera_frame
tf::Transform ModeledObject::transformWithOffset()
{
    // Transform from camera_frame to pose without offset
    tf::Transform tr_0;
    tr_0 = poseToTransform(pose_from_camera_);
    ROS_DEBUG_STREAM("Transform tr_0 - from camera_frame to pose without offset:\n" << pose_from_camera_);

    // Transform from pose without offset to the one with offset
    tf::Transform tr_1;
    tr_1.setOrigin(tf::Vector3(offsets[0],offsets[1],offsets[2]));
    tf::Quaternion q(0,0,0,1);
    tr_1.setRotation(q);

    geometry_msgs::Pose pose = transformToPose(tr_1);
    ROS_DEBUG_STREAM("Transform tr_1 - from pose without offset to the one with offset:\n" << pose);

    Eigen::Affine3d e_0;
    tf::transformTFToEigen(tr_0, e_0);
    Eigen::Affine3d e_1;
    tf::transformTFToEigen(tr_1, e_1);

    Eigen::Affine3d e;
    tf::Transform tr;
    e = e_0 * e_1;
    tf::transformEigenToTF(e, tr);

    tf::poseEigenToMsg(e, pose);
    ROS_DEBUG_STREAM("Eigen:\n" << pose);
    tf::poseEigenToMsg(e_0, pose);
    ROS_DEBUG_STREAM("Eigen_0:\n" << pose);
    tf::poseEigenToMsg(e_1, pose);
    ROS_DEBUG_STREAM("Eigen_1:\n" << pose);

    normalizeTransform(&tr);

    geometry_msgs::Pose pose_print = transformToPose(tr);
    ROS_DEBUG_STREAM("Transform from camera_frame to pose with offset:\n" << pose_print);

    return tr;
}
