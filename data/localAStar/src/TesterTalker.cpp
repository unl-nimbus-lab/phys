#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <sstream>
#include <math.h>

#include "MapMaker.h"
#include "MakeMarker.h"
#include "node.h"
#include "RobotSteering.h"

///global variables
geometry_msgs::Pose goalPosition;
geometry_msgs::PoseStamped globalGoalPosition;
geometry_msgs::Pose currentPosition;
geometry_msgs::Pose firstPosition;
float firstRotation;
const double tf::Transformer::DEFAULT_CACHE_TIME = 10.0;
laser_geometry::LaserProjection projector;
//ros::Publisher pointPub;
ros::Publisher markerViz;
ros::Publisher cmd_vel;
float robotWidth = 1.0;//ważny parametr grubości naszego ulubieńca najlepiej podawać wraz z zapasem odległości od przeszkód
float plannerRange = 5;
bool Ruszaj_Batmobilu = false;
bool Skanuj_Batmobilu = false;
bool turn2Target = false;
tf::TransformListener* listener;

myMap* occupancyMap = NULL;
std::string path = "";
sensor_msgs::PointCloud obst_cloud;
std::vector<geometry_msgs::Point> path_Point;

///function dectarations
void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scanMsg);
void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
void UpdateCurrentPosition(geometry_msgs::Pose &newPose);
std::vector <geometry_msgs::Point> Find_Points(sensor_msgs::PointCloud* cloud, const sensor_msgs::LaserScan::Ptr& scan);
bool isPathFree(geometry_msgs::Pose there, sensor_msgs::PointCloud* obstacles);
void BatmanFindPath(geometry_msgs::PoseStamped transformed_goalMsg);
geometry_msgs::PoseStamped transformGoal(geometry_msgs::PoseStamped goalMsg);
float angleFromPose(geometry_msgs::Pose pose);
///main
int main(int argc, char **argv)
{
    ros::init(argc, argv, "LocalPlanner");

    ros::NodeHandle n;
    ros::Subscriber subScan = n.subscribe("/batman/scan", 100, ScanCallback);
    ros::Subscriber goalSub = n.subscribe("/move_base_simple/goal", 20, GoalCallback);
    //pointPub = n.advertise<geometry_msgs::PoseStamped>("/path/chwilowy", 1000);
    cmd_vel = n.advertise<geometry_msgs::Twist>("/batman/aria/cmd_vel", 1);
    markerViz = n.advertise<visualization_msgs::Marker>("/markers", 1000);
    listener = new tf::TransformListener();
    ros::Rate loop_rate(10);

    RobotSteering RS;

    int nextPoint = 0;
    geometry_msgs::Twist twist;
    while (ros::ok())
    {
        UpdateCurrentPosition(currentPosition);

        if(turn2Target||Ruszaj_Batmobilu){

            geometry_msgs::PoseStamped tr_goal = transformGoal(globalGoalPosition);
            double angle;
            double dist;


            if(Ruszaj_Batmobilu){

                tf::Quaternion quat;
                quat.setX(currentPosition.orientation.x);
                quat.setY(currentPosition.orientation.y);
                quat.setZ(currentPosition.orientation.z);
                quat.setW(currentPosition.orientation.w);
                tf::Matrix3x3 m(quat);
                double yaw, pitch, roll;
                m.getEulerYPR(yaw, pitch, roll);

                std::cout<<"yaw: "<<yaw<<" pitch: "<<pitch<<" roll: "<<roll<<std::endl;



                angle = std::atan2((path_Point[nextPoint].y-currentPosition.position.y), (path_Point[nextPoint].x-currentPosition.position.x))
                        -yaw;
                dist = std::sqrt(std::pow(path_Point[nextPoint].x-currentPosition.position.x, 2)
                                +std::pow(path_Point[nextPoint].y-currentPosition.position.y, 2));

                std::cout<<"kat obliczony ruszaj!: "<<angle<<" odleglosc: "<<dist<<std::endl;

                if(dist<RS.posTolerance){
                    nextPoint++;
                    if(nextPoint==path_Point.size()){
                        dist = std::sqrt(std::pow(path_Point[nextPoint-1].x-globalGoalPosition.pose.position.x, 2)
                                        +std::pow(path_Point[nextPoint-1].y-globalGoalPosition.pose.position.y, 2));
                        if(dist<RS.posTolerance*2){
                            turn2Target = false;
                        }
                        Ruszaj_Batmobilu=false;
                        path_Point.clear();
                        nextPoint = 0;
                    }
                    std::cout<<"oba zerowane bo dojechane"<<std::endl;
                    twist.linear.x = 0;
                    twist.angular.z = 0;
                }else{

                    twist.linear.x = RS.calculateLinearSpeed(dist, angle);
                    twist.angular.z = RS.calculateAngularSpeed(dist, angle);
                    std::cout<<"oba parametry "<<twist.linear.x<<" " <<twist.angular.z<<std::endl;
                }
            }else{
                angle = std::atan2(tr_goal.pose.position.y, tr_goal.pose.position.x);
                dist = tr_goal.pose.position.x*tr_goal.pose.position.x+tr_goal.pose.position.y*tr_goal.pose.position.y;
                std::cout<<"kat obliczony: "<<angle<<std::endl;
                if(std::abs(angle)>0.05){
                    twist.linear.x = 0;
                    twist.angular.z = RS.calculateAngularSpeed(0, angle);
                }else{
                    if(path_Point.size()==0){
                        Skanuj_Batmobilu = true;
                    }else{
                        Ruszaj_Batmobilu = true;
                    }
                }
            }
        }else{
            twist.linear.x = 0;
            twist.angular.z = 0;
        }

        cmd_vel.publish(twist);

        ros::spinOnce();

        loop_rate.sleep();
    }
    twist.linear.x = 0;
    twist.angular.z = 0;
    cmd_vel.publish(twist);
    delete listener;
    return 0;
}


void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scanMsg)
{
    if(Skanuj_Batmobilu){
        //int scanNumber = scanMsg->scan_time / scanMsg->time_increment;
        float maxValue = 5.5;

        /*sensor_msgs::LaserScan::Ptr newMsg (new sensor_msgs::LaserScan(*scanMsg));
        for(int i=0;i<scanMsg->ranges.size();i++){
            if(std::isnan(scanMsg->ranges[i])) {
                newMsg->ranges[i] = maxValue;
            }
        }*/
        sensor_msgs::PointCloud cloud;
        //sensor_msgs::LaserScan::ConstPtr ptr (newMsg);

        projector.projectLaser(*scanMsg, cloud);
        //to nie jest miejsce na wywołanie tej funkcji, ale do testu się nadaje
       // isPathFree(goalPosition, &cloud);
        obst_cloud = cloud;
        FillOccupancyMap2(occupancyMap, &cloud, 0.1, maxValue);
        occupancyMap = mask(occupancyMap, 4);

        //if(Skanuj_Batmobilu){
            std::cout<< "Goal position x: "<<globalGoalPosition.pose.position.x
                    << ", y: "<<globalGoalPosition.pose.position.y<<std::endl;
            geometry_msgs::PoseStamped tr_goal = transformGoal(globalGoalPosition);
            std::cout<<tr_goal.header.frame_id<<" x: "<<tr_goal.pose.position.x<<" y: "<<tr_goal.pose.position.y<<std::endl<<std::endl;;
            BatmanFindPath(tr_goal);

        Skanuj_Batmobilu = false;
        firstPosition.position.x = currentPosition.position.x;
        firstPosition.position.y = currentPosition.position.y;
        firstPosition.position.z = currentPosition.position.z;
        firstRotation = angleFromPose(currentPosition);


        //Find_Points(&cloud, newMsg);
    }
    //drawMap(occupancyMap, markerViz);
}

void TfCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{

}
geometry_msgs::PoseStamped transformGoal(geometry_msgs::PoseStamped goalMsg){
    //przeliczam goal na współrzędne robota
    tf::Stamped<tf::Pose> goal, transformed_goal;
    geometry_msgs::PoseStamped transformed_goalMsg;
    goalMsg.header.frame_id = "batman/odom";
    goalMsg.header.stamp = ros::Time::now();

    tf::poseStampedMsgToTF(goalMsg, goal);
    try
    {
        listener->waitForTransform("batman/base_laser_link", goal.frame_id_, ros::Time::now(), ros::Duration(3.0));
        listener->transformPose("batman/base_laser_link", goal, transformed_goal);

        tf::poseStampedTFToMsg(transformed_goal, transformed_goalMsg);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    return transformed_goalMsg;
}
void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    globalGoalPosition= *goalMsg;
    geometry_msgs::PoseStamped transformed_goalMsg = transformGoal(*goalMsg);

    ///copy the given goal to global variable
    memcpy(&goalPosition, &(transformed_goalMsg.pose), sizeof(goalPosition));

    float x = transformed_goalMsg.pose.position.x;
    float y = transformed_goalMsg.pose.position.y;
    float z = transformed_goalMsg.pose.position.z;
    printf("goal: x, y, z: %f, %f, %f\n", x, y, z);


    if(x*x+y*y<plannerRange*plannerRange){
        turn2Target = true;
        Ruszaj_Batmobilu = false;
        Skanuj_Batmobilu = false;
    }
    else{
        std::cout<<"Cel poza zasięgiem planera ("<<plannerRange<<" m)!"<<std::endl;
    }

    /*std::cout<< "Goal position : id: "<<globalGoalPosition->header.frame_id
            << ", x: "<<globalGoalPosition->pose.position.x
            << ", y: "<<globalGoalPosition->pose.position.y<<std::endl;
    geometry_msgs::PoseStamped tr_goal = transformGoal(globalGoalPosition);
    std::cout<<tr_goal.header.frame_id<<" x: "<<tr_goal.pose.position.x<<" y: "<<tr_goal.pose.position.y<<std::endl<<std::endl;;
    BatmanFindPath(tr_goal);*/
}
void BatmanFindPath(geometry_msgs::PoseStamped transformed_goalMsg){
    float x = transformed_goalMsg.pose.position.x;
    float y = transformed_goalMsg.pose.position.y;

    float blockSize = occupancyMap->GetBlockSize();
    int xSize = occupancyMap->GetXSize();

    int xi = int((x) / blockSize);
    int yi = int((y) / blockSize + xSize - 1);
    printf("goal in map : %d, %d \n", xi, yi);
    std::string path = pathFind(0, xSize - 1, xi, yi, occupancyMap);
    //VisualizePath(path, occupancyMap, markerViz);
    if(path.size()>0)
    {
        std::vector<geometry_msgs::Point> vecPath = string2Vectors(path, occupancyMap);
        vecPath = PathShortener(vecPath, &obst_cloud, robotWidth);
        //for(int i=0;i<vecPath.size();i++){
        //    makeMarker(markerViz, 10000+i, vecPath[i].x, vecPath[i].y,(float)0.0,(float)1.0,(float)1.0,(float)1.0);
        //}

        //get all the points and transform them to an odom
        for(int i = 0; i<vecPath.size(); i++){

            geometry_msgs::PointStamped point, transPoint;
            point.header.frame_id = "batman/base_laser_link";
            point.header.stamp = ros::Time::now();
            point.point = vecPath[i];
            try
            {
                listener->waitForTransform("batman/odom", "batman/base_laser_link", ros::Time::now(), ros::Duration(3.0));
                listener->transformPoint("batman/odom", point, transPoint);
            }
            catch (tf::TransformException ex)
            {

                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
            std::cout<<"wpisuje"<<transPoint.point.x<<" "<<transPoint.point.y<<std::endl;

            vecPath[i] = transPoint.point;
        }

        //zapisz dzieło do zmiennej globalnej
        path_Point = vecPath;
        for(int i=0;i<vecPath.size();i++){
            makeMarkerGlobal(markerViz, 11000+i, vecPath[i].x, vecPath[i].y,(float)1.0,(float)0.0,(float)1.0,(float)1.0);
        }
    }
}


void UpdateCurrentPosition(geometry_msgs::Pose &newPose)
{
    tf::StampedTransform transform;

    try
    {
        listener->waitForTransform("batman/odom", "batman/base_link", ros::Time::now(), ros::Duration(3.0));
        listener->lookupTransform("batman/odom", "batman/base_link", ros::Time::now(), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    memcpy(&(newPose.position), transform.getOrigin(), sizeof(newPose.position));
    memcpy(&(newPose.orientation), transform.getRotation(), sizeof(newPose.orientation));

    ///printf("New pose: %f, %f, %f\n", newPose.position.x, newPose.position.y, newPose.position.z);
}
/*std::vector <geometry_msgs::Pose> Find_Path(){
    std::vector <geometry_msgs::Pose> somepath;
    return somepath;
}
//miało znaleźć punkty pośrednie do jazdy
std::vector <geometry_msgs::Point> Find_Points(sensor_msgs::PointCloud* cloud, const sensor_msgs::LaserScan::Ptr&  scan){
    std::vector <geometry_msgs::Point> points;
    int center = 0, points_num = 0;
    float centerAngle = 0.0;


    centerAngle = (scan->angle_min) ;
    std::cout<<"min "<< scan->angle_min <<"  incr " << scan->angle_increment << "center " << centerAngle
            << " max "<< scan->angle_max<<std::endl;


    center = int(std::abs( centerAngle / scan->angle_increment));

    std::cout<<center<<" ("<<cloud->points[center].x<<", "<<cloud->points[center].y<<")"<<std::endl;
    std::cout<<scan->ranges[center]<<std::endl<<std::endl;
    /*for(int i = 0; i<cloud->points.size(); i++){
        //kończy pętle, gdy dojdzie do pustych punktów
        if(cloud->points[i].x==0.0 && obstacles->points[i].y==0.0){
            break;
        }
        if(i>0){
            if(obstacles->points[i-1].y*obstacles->points[i].y<0){
                center=i;
                l++;
            }
        }
    }* /
    return points;
}*/

//funkcja posiada dwa założenia:
//  a) chmura i punkt docelowy są w układzie robota
//  b) cel jest dokładnie przed robotem
bool isPathFree(geometry_msgs::Pose there, sensor_msgs::PointCloud* obstacles){
    //przy spełnionych założeniach x jest wystarczający
    float distance = std::sqrt(there.position.x*there.position.x+there.position.y*there.position.y)+robotWidth/2;

    bool isFree = true;
    //int k = 0;
    //int l = 0;
    //ogólnie oś x - do przodu, oś y - w bok(jeśli wierzyć rvizowi, to w lewo), oś z do góry
    for(int i =0;i<obstacles->points.size();i++){
        //kończy pętle, gdy dojdzie do pustych punktów
        if(obstacles->points[i].x==0.0 && obstacles->points[i].y==0.0){
            break;
        }
        //sprawdza, czy punkt nie znajduje się za robotem
        if(obstacles->points[i].x<0){
           //nic nie robi, jak jest za robotem - zakres skanera zdaje się być większy niż 180 stopni
        }else{
            if(obstacles->points[i].x<distance && std::abs(obstacles->points[i].y)<robotWidth/2){
                isFree = false;
               // std::cout<<"punkt na drodze nr "<<i<<std::endl;
                ///makeMarker(markerViz, i, obstacles->points[i].x, obstacles->points[i].y, 1,1,1);
            }
        }
        /*
        //znajduje punkt najbardziej z przodu robota
        if(i>0){
            if(obstacles->points[i-1].y*obstacles->points[i].y<0){
                k=i;
                l++;
            }
        }
        */
    }
    //std::cout<<k<<" k"<<obstacles->points[k]<<" k+1 "<<obstacles->points[k+1]<<" k-1 "<<obstacles->points[k-1]<<std::endl;
    //std::cout<<l<<std::endl;

    /*
    //publikuje piękną strzałkę w rvizie :D
    geometry_msgs::PoseStamped position;
    position.pose.position.x = obstacles->points[k].x;
    position.pose.position.y = obstacles->points[k].y;
    position.header.stamp.now();
    position.header.frame_id = "batman/base_laser_link";
    pointPub.publish(position);
    */

    /* ważny test, który wykazał, że jak spotkamy punkt z dwoma zerami, to do końca tablicy wszystkie już takie będą, ale przy każdym skanie zera zaczynają się gdzie indziej
    bool zero=false;
    for(int i =0;i<obstacles->points.size();i++){
        if(obstacles->points[i].x==0.0 && obstacles->points[i].y==0.0)
        {
            if(!zero)
                std::cout<<i<<std::endl;
            zero = true;
        }else{
            if(zero){
                 std::cout<<obstacles->points[i];
            }
        }
    }*/
    return isFree;
}

float angleFromPose(geometry_msgs::Pose pose){
    tf::Quaternion quat2;
    quat2.setX(pose.orientation.x);
    quat2.setY(pose.orientation.y);
    quat2.setZ(pose.orientation.z);
    quat2.setW(pose.orientation.w);
    tf::Matrix3x3 m2(quat2);
    double yaw2, pitch2, roll2;
    m2.getEulerYPR(yaw2, pitch2, roll2);
    return yaw2;
}
