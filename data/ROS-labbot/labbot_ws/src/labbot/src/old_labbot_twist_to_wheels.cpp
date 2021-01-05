#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

class Jazda
{
  public:
    Jazda();
  private:
    void Callback(const geometry_msgs::Twist::ConstPtr& move);
    ros::NodeHandle n;
    float vmax, lv, rv, scale_ang;
    ros::Publisher lvel_pub;
    ros::Publisher rvel_pub;
    ros::Subscriber twist_sub;
};

Jazda::Jazda()
{
  vmax=0.2; //prędkość maksymalna w m/s
  scale_ang=0.5; //skalowanie obrotu
  lv=0;  
  rv=0;
  lvel_pub = n.advertise<std_msgs::Float32>("lwheel_vtarget", 1);
  rvel_pub = n.advertise<std_msgs::Float32>("rwheel_vtarget", 1);
  std_msgs::Float32 left;
  std_msgs::Float32 right;

  ros::Rate r(50.0);    //pętla wykonuję się w częstotliwości 50Hz
  while(n.ok()){
    ros::spinOnce(); 
    //nasłuchujemy temat Twist z np. joya
    twist_sub = n.subscribe<geometry_msgs::Twist>("Twist", 1, &Jazda::Callback, this);
 
    left.data=lv;
    right.data=rv;

    lvel_pub.publish(left);  //publikujemy prędkości lewego
    rvel_pub.publish(right);  //i prawego koła w m/s

    r.sleep();
  }
}

void Jazda::Callback(const geometry_msgs::Twist::ConstPtr& move)
{
  lv=move->linear.x*vmax-move->angular.z*vmax*scale_ang;
  rv=move->linear.x*vmax+move->angular.z*vmax*scale_ang;
  //sprawdzamy czy nie przekroczyliśmy prędkości maksymalnej
  if(lv>vmax)
    lv=vmax;
  if(lv<-vmax)
    lv=-vmax;
  if(rv>vmax)
    rv=vmax;
  if(rv<-vmax)
    rv=-vmax;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "twist_to_wheels_node");
  Jazda jazda;
}
