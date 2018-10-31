#ifndef MODELEDOBJECT_H
#define MODELEDOBJECT_H

// ROS
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometric_shapes/solid_primitive_dims.h>

// Update pose in Rviz
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "romeo_moveit_actions/metablock.hpp"
//#include <metablock.hpp>

class ModeledObject
{
public:
    ModeledObject(ros::NodeHandle *nh_,
                  moveit_visual_tools::MoveItVisualToolsPtr &visual_tools,
                  const std::string model_name,
                  const std::string rviz_name,
                  const geometry_msgs::PoseStamped pose,
                  const bool is_reference_object,
                  const bool camera_positioned,
                  const bool verbose);
    ~ModeledObject();

    void updatePose(geometry_msgs::PoseStamped pose);

    std::string getModelName()
    {
        return model_name_;
    }

    bool getHasObjectPose()
    {
        return has_object_pose_;
    }

    void setNotObjectPose()
    {
        has_object_pose_ = false;
    }

    void setVerbose(bool verbose)
    {
        verbose_ = verbose;
    }

    void setCameraPositioned(bool camera_positioned)
    {
        camera_positioned_ = camera_positioned;
    }

    static geometry_msgs::Pose transformToPose(tf::Transform tr)
    {
        geometry_msgs::Pose pose;

        tf::Quaternion q(tr.getRotation().x(), tr.getRotation().y(), tr.getRotation().z(), tr.getRotation().w());
        q.normalize();
        tr.setRotation( q );

        pose.position.x = tr.getOrigin().x();
        pose.position.y = tr.getOrigin().y();
        pose.position.z = tr.getOrigin().z();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        return pose;
    }

    static tf::Transform poseToTransform(geometry_msgs::Pose pose)
    {
        tf::Transform tr;

        double tx = pose.position.x;
        double ty = pose.position.y;
        double tz = pose.position.z;
        double qx = pose.orientation.x;
        double qy = pose.orientation.y;
        double qz = pose.orientation.z;
        double qw = pose.orientation.w;

        tr.setOrigin(tf::Vector3(tx,ty,tz));
        tf::Quaternion q(qx,qy,qz,qw);
        q.normalize();
        tr.setRotation( q );

        return tr;
    }

    static void normalizeTransform(tf::Transform *tr)
    {
        tf::Quaternion q(tr->getRotation().x(), tr->getRotation().y(), tr->getRotation().z(), tr->getRotation().w());
        q.normalize();
        tr->setRotation( q );
    }

    MetaBlock *block_;
private:
    void publishTransforms();
    tf::Transform transformWithOffset();

    bool verbose_;

    boost::shared_ptr<boost::thread> transform_thread_;

    std::string models_directory_;
    std::string rviz_name_;
    std::string model_name_;
    std::string model_folder_;
    std::string model_filename_;

    std::string object_frame_id_;
    std::string camera_frame_id_;
    std::string base_link_;

    moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

    bool has_object_pose_;
    bool camera_positioned_;

    geometry_msgs::Pose pose_from_camera_;

    bool pose_ajusted_;
    bool first_update_done_;
    tf::Transform tr_ajusted_;

    std::vector<float> offsets;

    bool is_reference_object_;
};

#endif // MODELEDOBJECT_H
