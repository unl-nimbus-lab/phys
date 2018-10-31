#include "ros/ros.h"
#include "geometry_msgs/Point32.h"
#include "std_msgs/Bool.h"
#include <stdio.h>

#define PI 3.14159265f

#define DEBUG 1
#define ARQ_DEBUG 1

#define DISTANCIA_MAXIMA 3.0f
#define DISTANCIA_MINIMA 1.0f
#define VELOCIDADE_LINEAR 1.1f
#define VELOCIDADE_LINEAR_APROX 0.70f

//Variaveis Globais-------------------------------------------------------------------------------------------------------

geometry_msgs::Point32 DistanciaCamera;
//geometry_msgs::Point32 DistanciaInicialCamera;

geometry_msgs::Point32 velocidadeRobo;

static bool enable = false;

#if defined(ARQ_DEBUG)
  FILE *arqCONE;
#endif

//Funcoes ---------------------------------------------------------------------------------------------------------------

void distanciaCallback(const geometry_msgs::Point32::ConstPtr& msg);
void enableCallback(const std_msgs::Bool::ConstPtr& msg);
void calcularVelocidades(void );

//-----------------------------------------------------------------------------------------------------------------------
void distanciaCallback(const geometry_msgs::Point32::ConstPtr& msg){

  DistanciaCamera.x = msg->x;
  DistanciaCamera.y = msg->y;
  DistanciaCamera.z = msg->z;

}
void enableCallback(const std_msgs::Bool::ConstPtr& msg){

  enable = msg -> data;
}

void calculaVelocidades2 (void) {
  
  float k_linear = 0.7;
  static bool fase3 = false;

  if (!enable || (DistanciaCamera.z==0 && !fase3)){
    fase3 = false;
    velocidadeRobo.x = 0;
    velocidadeRobo.z = 0;
    return;
  }
  if (DistanciaCamera.x > DISTANCIA_MAXIMA) {
    fase3 = false;
    velocidadeRobo.x = VELOCIDADE_LINEAR;
    velocidadeRobo.z = -15*PI/180*(DistanciaCamera.y);
  }  
  else if (DistanciaCamera.x > DISTANCIA_MINIMA){
    fase3 = false;
    velocidadeRobo.x = k_linear*(DistanciaCamera.x);
    velocidadeRobo.z = -15*PI/180*(DistanciaCamera.y);
  }
  else if (DistanciaCamera.x < DISTANCIA_MINIMA){
    fase3 = true;
    velocidadeRobo.x = VELOCIDADE_LINEAR_APROX;
    velocidadeRobo.z = -15*PI/180*(DistanciaCamera.y);
  }

#if defined(ARQ_DEBUG)
  fprintf(arqCONE,"%f %f %f %f\n",velocidadeRobo.x,velocidadeRobo.z, DistanciaCamera.x, DistanciaCamera.y);
#endif

#if defined(DEBUG)
  #warning Atencao! DEBUG esta definido
  ROS_INFO("%f %f %f %f \n",velocidadeRobo.x,velocidadeRobo.z, DistanciaCamera.x, DistanciaCamera.y);
#endif

}

int main(int argc, char **argv)
{

#if defined(ARQ_DEBUG)
  #warning Atencao! ARQ_DEBUG esta definido
  arqCONE = fopen("feedbackCONE.txt", "w");
  //fopen("/home/pi/Documents/feedbackCONE.txt", "w");
#endif

  ros::init(argc, argv, "controle_camera");

  ros::NodeHandle n;
 
  ros::Subscriber subDistancia = n.subscribe("cone_position", 1000, distanciaCallback);
  ros::Subscriber subEnable = n.subscribe("follow_camera", 1000, enableCallback);

  ros::Publisher pubVelocidade = n.advertise<geometry_msgs::Point32>("velocity", 1000);

  ros::Rate loop_rate(100);

  while (ros::ok()){

    //Se estiver no modo seguir trajetoria
    if (enable){
      calculaVelocidades2();
      pubVelocidade.publish(velocidadeRobo);
    
    }
    else {
      velocidadeRobo.x = 0;
      velocidadeRobo.z = 0;
      pubVelocidade.publish(velocidadeRobo);
    }

    loop_rate.sleep();
    ros::spinOnce();
  
  }
#if defined(ARQ_DEBUG)
  fclose(arqCONE);
#endif
  return 0;
}


/**Funcao que calcula as velocidades para alcancar o ponto da camera*/
// void calculaVelocidades (const ros::TimerEvent&){

//   static float x = 0, y = 0, theta = 0;
//   static float vx = 0, vy = 0, w = 0;
//   static float periodo = 0;
//   static float amostras = 0, incremento = 0;
//   static float t =0;
//   static float erro1 = 0, erro2 = 0, erro3 = 0;
//   const float k1 = 1.2, k2 = 1.2, k3 = 1.2;
//   static int flag = 0;
//   static float distanciaRestantex = 0, distanciaRestantey = 0, distanciaRestantetheta = 0;

//   if (!enable){
//     vx = 0;
//     vy = 0;
//     w  = 0;
//     erro1 = 0;
//     erro2 = 0;
//     erro3 = 0;
//     return;
//   }
//   if (inicio){

//     x = (DistanciaCamera.x)*cos(DistanciaCamera.z *PI/180);
//     y = (DistanciaCamera.x)*sin(DistanciaCamera.z *PI/180);
    
//     theta = DistanciaCamera.z * PI/180;

//     if ((abs(x) >= abs(y)) && (abs(x) >= abs(theta))) {
//       periodo = x/VEL_VIRTUAL;
    
//       if(periodo < 0){
//         periodo = -periodo;
//       }

//       vx = x/periodo;
//       vy = y/periodo;
//       w = theta/periodo;
//     }
//     else if ((abs(y) > abs(x)) && (abs(y) >= abs(theta))){
//       periodo = y/VEL_VIRTUAL;
//       if(periodo < 0){
//         periodo = -periodo;
//       }
//       vx = x/periodo;
//       w = theta/periodo;
//       vy = y/periodo;
//     }
//     else if ((abs(theta) > abs(x))&&(abs(theta) > abs(y))) {
//       periodo = theta/VEL_VIRTUAL;
//       if(periodo < 0){
//         periodo = -periodo;
//       }
//       vx = x/periodo;
//       vy = y/periodo;
//       w = theta/periodo;
//     }

//     amostras = periodo/TEMPO_AMOSTRAGEM;
//     t = 0;
//     incremento = 1/amostras;

//     inicio = false;

//   }

//   if (t < 1){ 
      
//     distanciaRestantex = ((DistanciaInicialCamera.x)*cos(DistanciaInicialCamera.z *PI/180)) - t*vx;
//     distanciaRestantey = ((DistanciaInicialCamera.x)*sin(DistanciaInicialCamera.z *PI/180)) - t*vy;
//     distanciaRestantetheta = (DistanciaInicialCamera.z * PI/180) - t*w;
//     t += incremento;
//     flag = 1;
  
//   }else if (flag == 1){
  
//     flag = 0;
//     distanciaRestantex = 0;
//     distanciaRestantey = 0;
//     distanciaRestantetheta = 0;
      
//   }  

//   erro1 = DistanciaCamera.x - distanciaRestantex;
//   erro2 = DistanciaCamera.y - distanciaRestantey;
//   erro3 = DistanciaCamera.z*PI/180 - distanciaRestantetheta;

//   velocidadeRobo.x = (vx*cos(erro3)) + k1*erro1;
//   velocidadeRobo.z = w + (vx * k2 * erro2) + (k3 * sin(erro3));

// }
