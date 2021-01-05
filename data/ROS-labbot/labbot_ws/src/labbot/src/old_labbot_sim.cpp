#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>

class Jazda
{
  public:
    Jazda();
  private:
    void Callback1(const std_msgs::Float32::ConstPtr& v);
    void Callback2(const std_msgs::Float32::ConstPtr& v);
    ros::NodeHandle n;
    int lticks, rticks, delta_lticks, delta_rticks, ticks_meter;
    double dt;
    ros::Time current_time, last_time;
    ros::Publisher l_pub;
    ros::Publisher r_pub;
    ros::Subscriber l_sub;
    ros::Subscriber r_sub;
};

Jazda::Jazda()
{
  lticks=0;  //stan licznika enkodera lewego koła
  rticks=0;  //prawego enkodera
  delta_lticks=0;  //zmiana wartości licznika enk. lewego koła
  delta_rticks=0;  //i prawego
  ticks_meter=5000;
  l_pub = n.advertise<std_msgs::Int16>("lwheel", 1);
  r_pub = n.advertise<std_msgs::Int16>("rwheel", 1);
  std_msgs::Int16 left;
  std_msgs::Int16 right;

  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(10.0);  //pętla wykonuję się w częstotliwości 10Hz
  while(n.ok()){
    ros::spinOnce();
    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec(); // różnica czasu od ostatniej pętli
    //nasłuchujemy na temacie prędkości kół w m/s
    l_sub = n.subscribe<std_msgs::Float32>("lwheel_vtarget", 1, &Jazda::Callback1, this);
    r_sub = n.subscribe<std_msgs::Float32>("rwheel_vtarget", 1, &Jazda::Callback2, this);
 
    lticks+=delta_lticks;  //wyznaczamy obecną wartość liczników enkoderów obu kół
    rticks+=delta_rticks;

    left.data=lticks;
    l_pub.publish(left);  //i publikujemy wyznaczone wartości

    right.data=rticks;
    r_pub.publish(right);

    last_time = current_time;
    r.sleep();
  }
}

void Jazda::Callback1(const std_msgs::Float32::ConstPtr& v)
{
  delta_lticks=v->data*ticks_meter*dt;  //wyznaczamy zmianę
}

void Jazda::Callback2(const std_msgs::Float32::ConstPtr& v)
{
  delta_rticks=v->data*ticks_meter*dt;  //wyznaczamy zmianę
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robotsim_node");
  Jazda jazda;
}
