#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>

#define MAIN_FREQ               60.0            //(Unit:HZ)
#define TF_REV_DELAY_T          5.0             //TF of MAP_ODOM frame receive start delay units: s
#define TF_REV_DELAY_CNT        (TF_REV_DELAY_T/MAIN_FREQ)
#define DF_ODOMETRY "odom"
#define ODOM_FRAME "odom"
#define ODOM_CHILD_FRAME "base_link"
#define FIX_Z_AXIS_FLAG         1
#define FIX_Z_HEIGHT            0.45            //(Unit:m)

//ROS Parameter
double main_freq_= MAIN_FREQ;
double tf_rev_delay_t_= TF_REV_DELAY_T;
std::string odometry_name_= DF_ODOMETRY;
std::string odom_frame_= ODOM_FRAME;
std::string odom_child_frame_= ODOM_CHILD_FRAME;
double fix_z_axis_flag_=FIX_Z_AXIS_FLAG;
double fix_z_height_=FIX_Z_HEIGHT;

//--TF /local_origin<---/robot_base data
int tf_rev_flag=0;
int tf_delay=0;
int tf_rev_delay_cnt=0;
double tf_x=0;
double tf_y=0;
double tf_z=0;
double tf_roll=0;
double tf_pitch=0;
double tf_yaw=0;
tf::Quaternion tf_quat;
double tf_old_x=0;
double tf_old_y=0;
double tf_old_z=0;
double tf_old_roll=0;
double tf_old_pitch=0;
double tf_old_yaw=0;

double vx=0.0;
double vy=0.0;
double vz=0.0;
double vroll=0.0;
double vpitch=0.0;
double vyaw=0.0;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vicon_odometry_node");
    ros::NodeHandle n("~");

    //Subscribe TF
    tf::TransformListener listener;

    //load ROS Parameter
    n.getParam("main_freq", main_freq_);
    n.getParam("tf_rev_delay_t", tf_rev_delay_t_);
    n.getParam("odometry_name", odometry_name_);
    n.getParam("odom_frame", odom_frame_);
    n.getParam("odom_child_frame", odom_child_frame_);
    n.getParam("fix_z_axis_flag", fix_z_axis_flag_);
    n.getParam("fix_z_height", fix_z_height_);

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>(odometry_name_, 50);
    tf::TransformBroadcaster odom_broadcaster;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    tf_rev_delay_cnt=tf_rev_delay_t_*main_freq_;
    ros::Rate r(main_freq_);
    while(n.ok())
    {
        //=============================================================================
        tf::StampedTransform tf_vicon_base;     //Receive tf (VICON) vicon <-- robot_base data
        if ( (tf_delay>=tf_rev_delay_cnt) )
        {
            //---------------------------------
            //TF vicon robot_base
            try{
                listener.lookupTransform("vicon", "robot_base", ros::Time(0), tf_vicon_base);
                tf_x=tf_vicon_base.getOrigin().x();
                tf_y=tf_vicon_base.getOrigin().y();
                if (fix_z_axis_flag_==0)
                    tf_z=tf_vicon_base.getOrigin().z();
                else
                    tf_z=fix_z_height_;
                tf_quat=tf_vicon_base.getRotation();
                tf::Matrix3x3(tf_quat).getRPY(tf_roll, tf_pitch, tf_yaw);
                current_time=ros::Time::now();
                tf_rev_flag=1;
            }
            catch (tf::TransformException ex){
                    ROS_ERROR("%s",ex.what());
            }
            //---------------------------------
            tf_delay=tf_rev_delay_cnt;
        }
        tf_delay++;


        if (tf_rev_flag==1)
        {
            double dt = (current_time - last_time).toSec();
            vx = (tf_x-tf_old_x) * dt;
            vy = (tf_y-tf_old_y) * dt;
            vz = (tf_z-tf_old_z) * dt;
            vroll =(tf_roll-tf_old_roll) * dt;
            vpitch = (tf_pitch-tf_old_pitch) * dt;
            vyaw = (tf_yaw-tf_old_yaw) * dt;

            geometry_msgs::Quaternion odom_quat;
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_link";

            odom_trans.transform.translation.x = tf_x;
            odom_trans.transform.translation.y = tf_y;
            odom_trans.transform.translation.z = tf_z;
            tf::quaternionTFToMsg(tf_quat, odom_quat);
            odom_trans.transform.rotation = odom_quat;
            odom_broadcaster.sendTransform(odom_trans);

            //next, we'll publish the odometry message over ROS
            nav_msgs::Odometry odom;
            odom.header.stamp = current_time;
            odom.header.frame_id = "odom";

            //set the position
            odom.pose.pose.position.x= tf_x;
            odom.pose.pose.position.y= tf_y;
            odom.pose.pose.position.z= tf_z;
            odom.pose.pose.orientation= odom_quat;

            //set the velocity
            odom.child_frame_id = "base_link";
            odom.twist.twist.linear.x= vx;
            odom.twist.twist.linear.y= vy;
            odom.twist.twist.linear.z= vz;
            odom.twist.twist.angular.x= vroll;
            odom.twist.twist.angular.y= vpitch;
            odom.twist.twist.angular.z= vyaw;

            //publish the message
            odom_pub.publish(odom);

            tf_old_x=tf_x;
            tf_old_y=tf_y;
            tf_old_z=tf_z;
            tf_old_roll=tf_roll;
            tf_old_pitch=tf_pitch;
            tf_old_yaw=tf_yaw;
            last_time = ros::Time::now();
            tf_rev_flag=0;
        }

        //=============================================================================
        ros::spinOnce();               // check for incoming messages
        r.sleep();
    }
}
