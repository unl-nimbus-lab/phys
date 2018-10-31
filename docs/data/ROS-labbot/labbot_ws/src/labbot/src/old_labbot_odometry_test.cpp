#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int16.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

double d_ltick, d_rtick, ltick=0, rtick=0;
int low_wrap, high_wrap;

void Callback1(const std_msgs::Int16::ConstPtr& enk1);
void Callback2(const std_msgs::Int16::ConstPtr& enk2);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry_tf_node");
  float base_width=0.29;  //odległość między kołami
  int tick_meter=687;//775.8832547454;  //liczba impulsów enkodera na jeden przejechany metr
  low_wrap=-32767*0.7;  //zmienne używane do zabezpieczenia przed
  high_wrap=32767*0.7;  //przepełnieniem zmiennych zliczających impulsy enkoderów

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);//będziemy publikować pozycję, prędkość i orientację w standardowym ros'owym temacie odom
  tf::TransformBroadcaster odom_broadcaster; //oraz w temacie tf dla navigation_stack
  ros::Subscriber l_sub;
  ros::Subscriber r_sub;

  double d_ls, d_rs; 
  double ds;
  double x=0.0, y=0.0, th=0.0; //pozycja startowa
  double dx, dy, dth, vx, vy, wth, dt;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(10.0);    //pętla wykonuję się w częstotliwości 10Hz
  while(n.ok()){
    ros::spinOnce();
    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();  //czas jaki upłynął od ostatniej pętli

    l_sub = n.subscribe<std_msgs::Int16>("lewy", 10, Callback1);  //nasłuchujemy tematy wartości liczników enkoderów kół
    r_sub = n.subscribe<std_msgs::Int16>("prawy", 10, Callback2);

    d_ls=d_ltick/tick_meter*1.0;  //wyznaczamy przejechaną zmianę odległości lewego
    d_rs=d_rtick/tick_meter*1.0;  //i prawego koła
    ds=(d_rs+d_ls)/2.0;  //wyznaczamy przejechaną zmianę odległości robota 
    dth=(d_ls-d_rs)/base_width*1.0;  //wyznaczamy zmianę w orientacji
    
    th+=dth;	//wyznaczamy nowe współrzędne położenia i orientacji
    dx=ds*cos(th+dth/2.0);
    dy=ds*sin(th+dth/2.0);
    x+=dx;
    y+=dy;

    vx=dx/dt*1.0;  //wyznaczamy prędkości liniowe w osiach współrzędnych x, y i obrotowe w z.
    vy=dy/dt*1.0;
    wth=dth/dt*1.0;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
 
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "myodom";
    odom_trans.child_frame_id = "mybase_link";
  
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
 
    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
 
    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "myodom";
 
    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
 
    //set the velocity
    odom.child_frame_id = "mybase_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = wth;
 
    //publish the message
    odom_pub.publish(odom);
 
    last_time = current_time;
    r.sleep();
  }
}

void Callback1(const std_msgs::Int16::ConstPtr& enk1)
{
  int wdol, wgore;
  if(enk1->data>high_wrap & ltick<low_wrap)//zabezpieczenie przed przepełnieniem
    wdol=1;
  else
    wdol=0;
  if(enk1->data<low_wrap & ltick>high_wrap)
    wgore=1;
  else
    wgore=0;
  d_ltick=enk1->data-ltick+wgore*2*32767-wdol*2*32767;//wyznaczamy różnicę między obecnym stanem licznika enkodera a stanem poprzednim
  ltick=enk1->data;
}

void Callback2(const std_msgs::Int16::ConstPtr& enk2)
{
  int wdol, wgore;
  if(enk2->data>high_wrap & rtick<low_wrap)//zabezpieczenie przed przepełnieniem
    wdol=1;
  else
    wdol=0;
  if(enk2->data<low_wrap & rtick>high_wrap)
    wgore=1;
  else
    wgore=0;
  d_rtick=enk2->data-rtick+wgore*2*32767-wdol*2*32767;//wyznaczamy różnicę między obecnym stanem licznika enkodera a stanem poprzednim
  rtick=enk2->data;
}

