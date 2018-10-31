#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/Point32.h"

#include <sstream>
#include <vector>
#include <math.h>

#define TEMPO_AMOSTRAGEM 0.1f
#define PI 3.14159265f

geometry_msgs::PoseStamped PosicaoAtual, PosicaoAnterior;

struct velocidade
{
    float dir, esq;
    
};

struct velocidade Vatual;
float velocidadex = 0, velocidadey = 0, velocidadet = 0;

void velocidade_atualCallback(const geometry_msgs::Point32::ConstPtr& msg){

    const float b1 = (0.06)/2, b2 = 0.06/0.075;

    Vatual.dir = msg->x *(2*PI);
    Vatual.esq = msg->y *(2*PI);

    velocidadex = b1*cos(PosicaoAtual.pose.orientation.z )*Vatual.dir + b1*cos(PosicaoAtual.pose.orientation.z)*Vatual.esq; 
    velocidadey = b1*sin(PosicaoAtual.pose.orientation.z )*Vatual.dir + b1*sin(PosicaoAtual.pose.orientation.z)*Vatual.esq; 
    velocidadet = b2*Vatual.dir - b2*Vatual.esq;
}

void CalculaPosicao(const ros::TimerEvent&){

    PosicaoAtual.pose.position.x += (velocidadex*TEMPO_AMOSTRAGEM);
    PosicaoAtual.pose.position.y += (velocidadey*TEMPO_AMOSTRAGEM);
    PosicaoAtual.pose.orientation.z += velocidadet*TEMPO_AMOSTRAGEM;

    float abs_theta = abs(PosicaoAtual.pose.orientation.z);

    while (abs_theta > PI){
      abs_theta = abs(PosicaoAtual.pose.orientation.z);
      if (abs_theta > 2*PI) {
        if (PosicaoAtual.pose.orientation.z > 0) {
         PosicaoAtual.pose.orientation.z -= 2*PI;
        }
        else if (PosicaoAtual.pose.orientation.z < 0) {
          PosicaoAtual.pose.orientation.z += 2*PI;
        }
      }
      else if (abs_theta > PI){
        if (PosicaoAtual.pose.orientation.z > 0) {
          PosicaoAtual.pose.orientation.z = 2*PI - PosicaoAtual.pose.orientation.z;

        }
        else if (PosicaoAtual.pose.orientation.z < 0) {
          PosicaoAtual.pose.orientation.z = 2*PI + PosicaoAtual.pose.orientation.z;
        } 

      }
    }

}
int main(int argc, char **argv)
{

  ros::init(argc, argv, "planejador");

  ros::NodeHandle n;

  ros::Publisher posicaoPub = n.advertise<geometry_msgs::PoseStamped>("position_odom", 1000);

  ros::Subscriber pegaVelocidade = n.subscribe("velocidade_atual", 1000, velocidade_atualCallback);

  ros::Timer temporizador = n.createTimer(ros::Duration(TEMPO_AMOSTRAGEM),CalculaPosicao, false);
  sleep(1);

  /*nav_msgs::Path trajeto;
  std::vector<geometry_msgs::PoseStamped> pose;
  geometry_msgs::PoseStamped auxPosition;*/

  ros::Rate loop_rate(100);

  PosicaoAtual.pose.position.x = 0;
  PosicaoAtual.pose.position.y = 0;
  PosicaoAtual.pose.orientation.z = 0;
  
  while (ros::ok())
  {
    posicaoPub.publish(PosicaoAtual);
    
    ros::spinOnce();

    loop_rate.sleep();

  }

  return 0;
}
