#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point32.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/Int16.h"
#include "raspberry_msgs/StampedFloat32.h"
#include <tf/transform_listener.h>
#include <math.h>
#include <stdio.h>

#define PI 3.14159265f

//#define ARQ_DEBUG 1
#define DEBUG 1
#define GAZEBO 1
#define ARDUINO 1
#define CONVERTER_COORD 1
#define TESTE_US 1

// Modos de operacao
#define PARA 0
#define SEGUIR_TRAJETORIA 1
#define APROXIMAR_CONE 2
#define SEGUIR_VELOCIDADE 3

#define TEMPO_AMOSTRAGEM 0.1f

#if defined(GAZEBO)
#define VELOCIDADE_MAXIMA 1.0f
#define VELOCIDADE_MAXIMA_APROX 0.7f
#define VELOCIDADE_MINIMA 0.15f
#define VELOCIDADE_MINIMA_ANGULAR 0.05f
#define VEL_VIRTUAL 1.0f
#define VELOCIDADE_MAX_ARDUINO 5.5f
#define VELOCIDADE_MAX_ARDUINO_APROX 3.0f
#define vel_min_arduino 1.5f
#endif

/*#if defined(ARDUINO)
#define VELOCIDADE_MAXIMA 5.5f
#define VELOCIDADE_MAXIMA_APROX 3.0f
#define VELOCIDADE_MINIMA 1.0f
#define VELOCIDADE_MINIMA_ANGULAR 1.0f
#define VEL_VIRTUAL 2.0f
#define VELOCIDADE_MAX_ARDUINO 5.5f
#define VELOCIDADE_MAX_ARDUINO_APROX 3.0f
#define vel_min_arduino 1.5f
#endif
*/
#define DISTANCIA_MAXIMA 1.0f
#define DISTANCIA_MINIMA 0.002f
#define NUM_US 11
#define DISTANCIA_CENTRO 0.1f

//Variaveis Globais-------------------------------------------------------------------------------------------------------

//valores Robo
struct Robo{
  float x,y,theta;
};

struct ultrassons {
  int id;
  float x, y, z;
};

std::vector<geometry_msgs::PoseStamped> pose;
geometry_msgs::PoseStamped auxPose;
geometry_msgs::Point32 VelocidadeRecebida;

#if defined(GAZEBO)
  #warning Atencao!Gazebo esta definido
  geometry_msgs::Twist velocidadeRobo;
#endif

#if defined(ARDUINO)
  #warning Atencao!Arduino esta definido
  raspberry_msgs::StampedFloat32 velocidadeArduinoEsquerda;
  raspberry_msgs::StampedFloat32 velocidadeArduinoDireita;
#endif

Robo roboAtual, roboDestino,roboReferencia, roboInicial;

bool parar = true;
bool novoSegmento = false;
bool obstaculo = false;
bool desviou = false;

static float velocidadeLinear = 0, velocidadeAngular = 0;

//conta o segmento de trajetoria que esta sendo seguido
static int trajetoriaAtual = 0;

//Parametros de adaptacao do controlador
//static float a1 =0, a2 = 0; 

//Parametros de erros
static float erro1 = 0, erro2 = 0, erro3=0;

/*Parametros do controle da ZVD*/
static float fk = 0, thetak = 0;

static bool inicio = false;

/*Flag que permite receber nova trajetoria, novas velocidades a serem seguidas*/
static int16_t enable = 0;

static float vel_max = 0, vel_max_arduino = 0;

float DIST_MAX[NUM_US] = {0.5,0.5,0.5,1,1,1,1,1,0.5,0.5,0.5};

float US[NUM_US] = {999,999,999,999,999,999,999,999,9999,999,999};

float SX[NUM_US] = {cos(87*PI/180)*DIST_MAX[0]+DISTANCIA_CENTRO,cos(69.6*PI/180)*DIST_MAX[1]+DISTANCIA_CENTRO,
cos(52.2*PI/180)*DIST_MAX[2]+DISTANCIA_CENTRO,cos(34.8*PI/180)*DIST_MAX[3]+DISTANCIA_CENTRO,
cos(17.4*PI/180)*DIST_MAX[4]+DISTANCIA_CENTRO,cos(0*PI/180)*DIST_MAX[5]+DISTANCIA_CENTRO,
cos(17.4*PI/180)*DIST_MAX[6]+DISTANCIA_CENTRO,cos(34.8*PI/180)*DIST_MAX[7]+DISTANCIA_CENTRO,
cos(52.2*PI/180)*DIST_MAX[8]+DISTANCIA_CENTRO,cos(69.6*PI/180)*DIST_MAX[9]+DISTANCIA_CENTRO,
cos(87*PI/180)*DIST_MAX[10]+DISTANCIA_CENTRO};

float SY[NUM_US] = {sin(87*PI/180)*DIST_MAX[0],sin(69.6*PI/180)*DIST_MAX[1],sin(52.2*PI/180)*DIST_MAX[2],
sin(34.8*PI/180)*DIST_MAX[3],sin(17.4*PI/180)*DIST_MAX[4],0*DIST_MAX[5],sin(-17.4*PI/180)*DIST_MAX[6],
sin(-34.8*PI/180)*DIST_MAX[7],sin(-52.5*PI/180)*DIST_MAX[8],sin(-69.6*PI/180)*DIST_MAX[9],sin(-87*PI/180)*DIST_MAX[10]};

#if defined(ARQ_DEBUG)
  #warning Atencao! ARQ_DEBUG esta definido
  FILE *arq, *arq2;
#endif

//Funcoes ---------------------------------------------------------------------------------------------------------------

void trajetoCallback(const nav_msgs::Path::ConstPtr& msg);
void enablePathCallback(const std_msgs::Int16::ConstPtr& msg);
void posicaoAtualCallback(const nav_msgs::Odometry::ConstPtr& msg);
void UltrassomCallback(const sensor_msgs::Range::ConstPtr& msg);
void calculaSegmento (void);
void RoboReferencia(const ros::TimerEvent&);
void controladorTrajetoria(void);
void controladorVelocidade(void);
void verificaObstaculos (tf::TransformListener &tfListener);

//-----------------------------------------------------------------------------------------------------------------------

/**Funcao do subscriber que recebe os pontos a serem seguidos*/
void trajetoCallback(const nav_msgs::Path::ConstPtr& msg)
{
 // if((enable != PARA) && (enable != SEGUIR_VELOCIDADE)){
    pose = msg->poses;
    trajetoriaAtual = 0;

#if defined(DEBUG)
    ROS_INFO("Posicao recebida");
#endif
 // }
}

void enablePathCallback(const std_msgs::Int16::ConstPtr& msg)
{
  enable = msg->data;

  if (enable == SEGUIR_TRAJETORIA) {
    vel_max = VELOCIDADE_MAXIMA;
    vel_max_arduino = VELOCIDADE_MAX_ARDUINO;
  }
  else if((enable == APROXIMAR_CONE) || (enable == SEGUIR_VELOCIDADE)){
    vel_max = VELOCIDADE_MAXIMA_APROX;
    vel_max_arduino = VELOCIDADE_MAX_ARDUINO_APROX;
  }
  else if (enable == PARA){
    parar = true;
  }

#if defined(DEBUG)
  if(enable == PARA){
    ROS_INFO("enable parar");
  }
  if (enable == SEGUIR_VELOCIDADE){
    ROS_INFO("enable seguir velocidade");
  }
  else if (enable == APROXIMAR_CONE) {
    ROS_INFO("enable aproximar cone");
  }
  else if (enable == SEGUIR_TRAJETORIA){
    ROS_INFO("enable seguir trajetoria");
  }
#endif

}

/**Posicao atual lida do robo*/
void posicaoAtualCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  geometry_msgs::Quaternion orientacao;

  double roll, pitch, yaw;
  
  tf::Quaternion quat;

  orientacao = msg->pose.pose.orientation;  
  
  //Tranforma a orientacao dada em quaternions para radianos 
  tf::quaternionMsgToTF(orientacao, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  //ROS_INFO("quaternions: x: %f y: %f z:%f w: %f", orientacao.x,orientacao.y,orientacao.z,orientacao.w);
  //ROS_INFO("yaw: %f pitch: %f roll: %f", yaw,pitch,roll);
  
#if defined(CONVERTER_COORD)

  #warning Atencao! CONVERTER_COORD esta definido!
  roboAtual.x = msg->pose.pose.position.y;
  roboAtual.y = msg->pose.pose.position.x;
  roboAtual.theta = PI/2 - yaw;
#else
  roboAtual.x = msg->pose.pose.position.x;
  roboAtual.y = msg->pose.pose.position.y;
  roboAtual.theta = yaw;
#endif

  //ROS_INFO("Posicao atual lida: x:%f y:%f theta: %f", roboAtual.x, roboAtual.y, roboAtual.theta);
}

/**Callback que recebe os valores lidos dos ultrassons*/
void UltrassomCallback(const sensor_msgs::Range::ConstPtr& msg){
  std::string usNames[] = {"/ultrasound1","/ultrasound2","/ultrasound3","/ultrasound4","/ultrasound5",
                                        "/ultrasound6","/ultrasound7","/ultrasound8","/ultrasound9","/ultrasound10",
                                        "/ultrasound11"};
  
  if (enable == SEGUIR_TRAJETORIA){
    for(int i=0; i<NUM_US;i++){
      if(usNames[i] == msg->header.frame_id){
        US[i] = msg->range;

#if defined(DEBUG)
        ROS_INFO("ultrassom %d leitura %f", i, US[i]);
#endif
      }
    }  
  }
}

void velocidadeCallback(const geometry_msgs::Point32::ConstPtr& msg){

  VelocidadeRecebida.x = msg->x;
  VelocidadeRecebida.y = msg->y;
  VelocidadeRecebida.z = msg->z;

}

/**Funcao que atualiza o segmento atual a ser realizado*/
void calculaSegmento (void) {

  static float aux_x = 0, aux_y = 0;
  static float gama = 0; /*angulo de inclinacao da reta de segmento de trajetoria*/

  //Indica que ele passou da regiao do seguimeto atual e ira prossegir para o proximo segmento ou que irá comecar o primeiro segmento  
#if defined(TESTE_US)
  if (enable == PARA){
    parar = false;
    inicio = false;
    erro1 = 0, erro2 = 0, erro3 = 0;
    return;
  }
#else
  if (enable == PARA){
    parar = true;
    inicio = false;
    erro1 = 0, erro2 = 0, erro3 = 0;
    return;
  }
#endif
  if (enable == SEGUIR_VELOCIDADE){
    parar = false;
    inicio  = false;
    return;
  }
  if ((((cos(gama)*(roboAtual.x - roboDestino.x))+(sin(gama)*(roboAtual.y - roboDestino.y))) > 0) || (trajetoriaAtual == 0)){
   
    if (!pose.empty()){
      auxPose = pose.front();
      pose.erase(pose.begin());

      aux_x = roboDestino.x;
      aux_y = roboDestino.y;

#if defined(CONVERTER_COORD)
      roboDestino.x = auxPose.pose.position.y;
      roboDestino.y = auxPose.pose.position.x;
#else        
      roboDestino.x = auxPose.pose.position.x;
      roboDestino.y = auxPose.pose.position.y;
#endif
      
      if (trajetoriaAtual == 0){
        gama = atan2((roboDestino.y - roboAtual.y),(roboDestino.x - roboAtual.x));  
      }
      else {
        gama = atan2((roboDestino.y - aux_y),(roboDestino.x - aux_x));
      }
      
      trajetoriaAtual++;

      roboReferencia = roboAtual;
      roboInicial = roboReferencia;
      //a1 = 0, a2 = 0; //Parametros do controlador adaptativo
      erro1 = 0, erro2 = 0, erro3 = 0;
      parar = false;
      inicio = true;

#if defined(DEBUG)
      ROS_INFO("Segmento atual %d:  x:%f y:%f",trajetoriaAtual, auxPose.pose.position.y,auxPose.pose.position.x);
#endif
#if defined(ARQ_DEBUG)
      fprintf(arq,"Segmento atual %d:  x:%f y:%f",trajetoriaAtual, auxPose.pose.position.y,auxPose.pose.position.x);
#endif

    }
    else{
      parar = true;
      inicio = false;
    }
  }

}

// Fazendo a trajetoria de referencia por meio de interpolacao
void RoboReferencia(const ros::TimerEvent&){

  static float vx = 0, vy = 0 , w = 0;
  static float periodo = 0, amostras = 0, incremento = 0;
  static float t =0; 
  float anguloReta = 0;
  static float x,y,theta;
  static int flag = 0;
  const float Kr = 3, Kt = -3;

  if ((inicio || desviou) && !parar) {

    desviou = false;
    
    x = roboDestino.x - roboInicial.x;
    y = roboDestino.y - roboInicial.y;
    
    anguloReta = atan2(y,x);

    theta = anguloReta - roboReferencia.theta;

    // Angulo de deslocamento fica entre -PI e PI
    float abs_theta = abs(theta);

    while (abs_theta > PI){
      abs_theta = abs(theta);
      if (abs_theta > 2*PI) {
        if (theta > 0) {
         theta -= 2*PI;
        }
        else if (theta < 0) {
          theta += 2*PI;
        }
      }
      else if (abs_theta > PI){
        if (theta > 0) {
          theta = 2*PI - theta;
        }
        else if (theta < 0) {
          theta = 2*PI + theta;
        } 

      }
    }
    /*while (abs(theta) > PI){
      if ((theta > PI) && (theta > 0)) {
        theta -= PI;
      }
      if ((theta < -PI) && (theta < 0)) {
        theta += PI;
      }
    }*/

    if ((abs(x) >= abs(y)) && (abs(x) >= abs(theta))) {
      periodo = x/VEL_VIRTUAL;
      if(periodo < 0){
        periodo = -periodo;
      }

      vx = x/periodo;
      vy = y/periodo;
      w = theta/periodo;
    }
    else if ((abs(y) > abs(x)) && (abs(y) >= abs(theta))){
      periodo = y/VEL_VIRTUAL;
      if(periodo < 0){
        periodo = -periodo;
      }
      vx = x/periodo;
      w = theta/periodo;
      vy = y/periodo;
    }
    else if ((abs(theta) > abs(x))&&(abs(theta) > abs(y))) {
      periodo = theta/VEL_VIRTUAL;
      if(periodo < 0){
        periodo = -periodo;
      }
      vx = x/periodo;
      vy = y/periodo;
      w = theta/periodo;
    }

    amostras = periodo/TEMPO_AMOSTRAGEM;
    t = 0;
    incremento = 1/amostras;

    inicio = false;

#if defined(DEBUG)
    ROS_INFO("periodo: %f amostras: %f t = %f", periodo, amostras, t);
#endif
#if defined(ARQ_DEBUG)
    fprintf(arq,"x: %f y: %f t = %f \n", x, y, theta);
    fprintf(arq,"periodo: %f amostras: %f incremento = %f\n", periodo, amostras, incremento);
    fprintf(arq,"velocidades: x: %f y :%f theta: %f \n", vx, vy, w);
#endif

  }

  if (!obstaculo) {
  
    if (t < 1){ 
  
      roboReferencia.x = roboInicial.x + t*x;
      roboReferencia.y = roboInicial.y + t*y;
      roboReferencia.theta = roboInicial.theta + t*theta;
      
      t += incremento;
      flag = 1;
    
    }else if (flag == 1){
    
      flag = 0;
      roboReferencia.x = roboInicial.x + x;
      roboReferencia.y = roboInicial.y + y;
      roboReferencia.theta = roboInicial.theta + theta;
        
    }  
  }

  if (!obstaculo) {
    /*velocidade linear  m/s */
    velocidadeLinear = cos(roboReferencia.theta)*vx + sin(roboReferencia.theta)*vy;
    /*velocidade em rad/s */
    velocidadeAngular = w;  
  }
  else {
    velocidadeLinear = /*cos(roboReferencia.theta)*vx + sin(roboReferencia.theta)*vy*/ velocidadeLinear - abs(Kt*(fk)*cos(thetak));
    velocidadeAngular = velocidadeAngular + Kr*(fk)*sin(thetak);
    
    roboReferencia.x = roboReferencia.x + (velocidadeLinear)*cos(roboReferencia.theta)*TEMPO_AMOSTRAGEM;
    roboReferencia.y = roboReferencia.y + (velocidadeLinear)*sin(roboReferencia.theta)*TEMPO_AMOSTRAGEM;
    roboReferencia.theta = roboReferencia.theta + velocidadeAngular*TEMPO_AMOSTRAGEM;

  }
  
  // deixa o angulo do robo entre -PI e PI
  /*while (abs(roboReferencia.theta) > PI){
    if ((roboReferencia.theta > PI) && (roboReferencia.theta > 0)) {
      roboReferencia.theta -= PI;
    }
    if ((roboReferencia.theta < -PI) && (roboReferencia.theta < 0)) {
      roboReferencia.theta += PI;
    }
  }*/

  float abs_thetaRobo = abs(roboReferencia.theta);

  while (abs_thetaRobo > PI){
    abs_thetaRobo = abs(roboReferencia.theta);
    if (abs_thetaRobo > 2*PI) {
      if (roboReferencia.theta > 0) {
        roboReferencia.theta -= 2*PI;
      }
      else if (roboReferencia.theta < 0) {
        roboReferencia.theta += 2*PI;
      }
    }
    else if (abs_thetaRobo > PI){
      if (roboReferencia.theta > 0) {
        roboReferencia.theta = 2*PI - roboReferencia.theta;
      }
      else if (roboReferencia.theta < 0) {
        roboReferencia.theta = 2*PI + roboReferencia.theta;
      } 
    }
  }

#if defined(TESTE_US)
  if (parar && !obstaculo) {
    velocidadeLinear = 0;
    velocidadeAngular = 0;
    vx = 0;
    vy = 0;
  }

#else
  if (parar) {
    velocidadeLinear = 0;
    velocidadeAngular = 0;
    vx = 0;
    vy = 0;
  }
#endif
}

/**Funcao do controlador de trajetoria para o robo real*/
void controladorTrajetoria(void /*const ros::TimerEvent&*/) {
  
  /*const float lambda = 1.47;
  //Parametros de ganho do controlador
  const float gama1 = 0.08;
  const float gama2 = 0.08;
  static float funcaoAuxiliar = 0;*/

  const float k1=1.2, k2=1.3, k3=1.2;
  const float b1 = 1/(0.06), b2 = 0.075/0.06;
  static float vf=0, wf=0; 
  //velocidades dadas às rodas do robo real
  static float velocidadeEsquerda = 0, velocidadeDireita = 0; 
  
  
  erro1 = cos(roboAtual.theta)*(roboReferencia.x - roboAtual.x) + sin(roboAtual.theta)*(roboReferencia.y - roboAtual.y);
  erro2 = -sin(roboAtual.theta)*(roboReferencia.x - roboAtual.x) + cos(roboAtual.theta)*(roboReferencia.y - roboAtual.y);
  erro3 = roboReferencia.theta - roboAtual.theta;
 
  vf = (velocidadeLinear * cos(erro3)) + (k1 *erro1);
  wf = velocidadeAngular + (velocidadeLinear * k2 * erro2) + (k3 * sin(erro3));

  //Calculo direto da velocidade
  
  velocidadeDireita = (b1*vf + b2*wf)/(2*PI);
  velocidadeEsquerda = (b1*vf - b2*wf)/(2*PI);

  //Controle adaptativo da trajetoria de velocidade 
  /*if (a2 > lambda){
    funcaoAuxiliar = 0; 
  }
  else {
    float fo = (gama2*wf*sin(erro3))/k2;

    funcaoAuxiliar = ((1 - (a2/lambda))*(1 - (a2/lambda))) + ((fo*fo) + 1 );

  }

  a1 = a1 + (gama1*erro1*vf);
  a2 = a2 + ((lambda/k2)*wf*sin(erro3)) + funcaoAuxiliar;
  
  // Transforma para rotacoes por segundo
  velocidadeDireita = (a1*vf + a2*wf)*(2*PI);
  velocidadeEsquerda = (a1*vf - a2*wf)*(2*PI);
  */

#if defined(GAZEBO)
  if (vf > vel_max){
    vf = vel_max;
  }
  else if (vf < -vel_max){
    vf = -vel_max;
  }
  if (wf > vel_max){
    wf = vel_max;
  }
  else if (wf < -vel_max){
    wf = -vel_max;
  }
#endif

#if defined(ARDUINO)
  
	float abs_esq = abs(velocidadeEsquerda), abs_dir = abs(velocidadeDireita);

	if ((abs_esq > vel_max_arduino) || (abs_dir > vel_max_arduino)){
		if (abs_esq > abs_dir ) {
			if (velocidadeDireita >= 0) {
				velocidadeDireita = (abs_dir/abs_esq)*vel_max_arduino;
			}
			else {
				velocidadeDireita = -(abs_dir/abs_esq)*vel_max_arduino;
			}
			if (velocidadeEsquerda > 0) {
				velocidadeEsquerda = vel_max_arduino;
			}
			else {
				velocidadeEsquerda = -vel_max_arduino;
			}
		}
		else if (abs_dir > abs_esq ) {
      if (velocidadeEsquerda >= 0) {
				velocidadeEsquerda = (abs_esq/abs_dir)*vel_max_arduino;
			}
			else {
				velocidadeEsquerda = -(abs_esq/abs_dir)*vel_max_arduino;
			}
			if (velocidadeDireita > 0) {
				velocidadeDireita = vel_max_arduino;
			}
			else {
				velocidadeDireita = -vel_max_arduino;
			}
		}	
		else {
			if (velocidadeEsquerda > 0) {
				velocidadeEsquerda = vel_max_arduino;
			}
			else {
				velocidadeEsquerda = -vel_max_arduino;
			}
			if (velocidadeDireita > 0) {
				velocidadeDireita = vel_max_arduino;
			}
			else {
				velocidadeDireita = -vel_max_arduino;
			}
		}	
	}
	
	else if ((abs_esq < vel_min_arduino) || (abs_dir < vel_min_arduino)){
		if (abs_esq < abs_dir ) {
      if (abs_esq != 0) {
  			if (velocidadeDireita >= 0) {
  				velocidadeDireita = (abs_dir/abs_esq)*vel_min_arduino;
  			}
  			else {
  				velocidadeDireita = -(abs_dir/abs_esq)*vel_min_arduino;
  			}
      }
      else {
        velocidadeDireita = vel_min_arduino;
      }
			if (velocidadeEsquerda > 0) {
				velocidadeEsquerda = vel_min_arduino;
			}
			else if (velocidadeEsquerda < 0){
				velocidadeEsquerda = -vel_min_arduino;
			}
      else {
        velocidadeEsquerda = 0; 
      }
		}
		else if (abs_dir < abs_esq ) {
      if (abs_dir != 0) {
  			if (velocidadeEsquerda >= 0) {
  				velocidadeEsquerda = (abs_esq/abs_dir)*vel_min_arduino;
  			}
  			else {
  				velocidadeEsquerda = -(abs_esq/abs_dir)*vel_min_arduino;
  			}
      }
      else {
        velocidadeEsquerda = vel_min_arduino;
      }
			if (velocidadeDireita > 0) {
				velocidadeDireita = vel_min_arduino;
			}
			else if (velocidadeDireita < 0){
				velocidadeDireita = -vel_min_arduino;
			}
      else {
        velocidadeDireita = 0;
      }
		}	
		else {
			if (velocidadeEsquerda > 0) {
				velocidadeEsquerda = vel_min_arduino;
			}
			else if (velocidadeEsquerda < 0) {
				velocidadeEsquerda = -vel_min_arduino;
			}
      else {
        velocidadeEsquerda = 0;
      }
			if (velocidadeDireita > 0) {
				velocidadeDireita = vel_min_arduino;
			}
			else if (velocidadeDireita < 0) {
				velocidadeDireita = -vel_min_arduino;
			}
      else {
        velocidadeDireita = 0;
      }
		}	
	}

#endif

  if (!parar /*|| obstaculo*/) {  // INCLUIR obstaculo para testar ZVD

    /*if ((!pose.empty()) && abs(velocidadeAngular) < 0.2){

      if ((vf < VELOCIDADE_MINIMA) && (vf>0)){
        vf = VELOCIDADE_MINIMA;
      }
      else if ((vf > (-VELOCIDADE_MINIMA)) && (vf < 0)){
        vf = -VELOCIDADE_MINIMA;
      }*/
      /*if ((wf < VELOCIDADE_MINIMA_ANGULAR) && (wf > 0)){
        wf = VELOCIDADE_MINIMA_ANGULAR;
      }
      else if ((wf > (-VELOCIDADE_MINIMA_ANGULAR)) && (wf < 0)){
        wf = -VELOCIDADE_MINIMA_ANGULAR;
      }*/
    
    //}

#if defined(GAZEBO)
    velocidadeRobo.linear.x = vf;
    velocidadeRobo.angular.z = wf;
#endif

#if defined(ARDUINO) 
    velocidadeArduinoEsquerda.data = velocidadeEsquerda;
    velocidadeArduinoDireita.data = velocidadeDireita;
#endif
  }
  else {
    vf = 0;
    wf = 0;

    velocidadeEsquerda = 0, velocidadeDireita = 0; 
    //funcaoAuxiliar = 0;

#if defined(GAZEBO)
    velocidadeRobo.linear.x = 0;
    velocidadeRobo.angular.z = 0;
#endif

#if defined(ARDUINO) 
    velocidadeArduinoEsquerda.data = 0;
    velocidadeArduinoDireita.data = 0;
#endif

  }

#if defined(ARQ_DEBUG)
  fprintf(arq2, "%f  %f %f %f \n", vf,wf, velocidadeEsquerda, velocidadeDireita);
#endif

}
void controladorVelocidade(void){

  const float b1 = 1/(0.06), b2 = 0.075/0.06;

  float velocidadeDireita = (b1*VelocidadeRecebida.x + b2*VelocidadeRecebida.z)/(2*PI);
  float velocidadeEsquerda = (b1*VelocidadeRecebida.x - b2*VelocidadeRecebida.z)/(2*PI);

  //ROS_INFO("Velocidades para o robo simulado: vf:%f wf:%f",vf,wf);
  //ROS_INFO("Velocidades para o robo real: vd:%f ve:%f",velocidadeDireita, velocidadeEsquerda);

  if (!parar) {
		float abs_esq = abs(velocidadeEsquerda), abs_dir = abs(velocidadeDireita);
		
    if ((abs_esq > vel_max_arduino) || (abs_dir > vel_max_arduino)){
      if (abs_esq > abs_dir ) {
        if (velocidadeDireita >= 0) {
          velocidadeDireita = (abs_dir/abs_esq)*vel_max_arduino;
        }
        else {
          velocidadeDireita = -(abs_dir/abs_esq)*vel_max_arduino;
        }
        if (velocidadeEsquerda > 0) {
          velocidadeEsquerda = vel_max_arduino;
        }
        else {
          velocidadeEsquerda = -vel_max_arduino;
        }
      }
      else if (abs_dir > abs_esq ) {
        if (velocidadeEsquerda >= 0) {
          velocidadeEsquerda = (abs_esq/abs_dir)*vel_max_arduino;
        }
        else {
          velocidadeEsquerda = -(abs_esq/abs_dir)*vel_max_arduino;
        }
        if (velocidadeDireita > 0) {
          velocidadeDireita = vel_max_arduino;
        }
        else {
          velocidadeDireita = -vel_max_arduino;
        }
      } 
      else {
        if (velocidadeEsquerda > 0) {
          velocidadeEsquerda = vel_max_arduino;
        }
        else {
          velocidadeEsquerda = -vel_max_arduino;
        }
        if (velocidadeDireita > 0) {
          velocidadeDireita = vel_max_arduino;
        }
        else {
          velocidadeDireita = -vel_max_arduino;
        }
      } 
    }
    
    else if ((abs_esq < vel_min_arduino) || (abs_dir < vel_min_arduino)){
      if (abs_esq < abs_dir ) {
        if (abs_esq != 0) {
          if (velocidadeDireita >= 0) {
            velocidadeDireita = (abs_dir/abs_esq)*vel_min_arduino;
          }
          else {
            velocidadeDireita = -(abs_dir/abs_esq)*vel_min_arduino;
          }
        }
        else {
          velocidadeDireita = vel_min_arduino;
        }
        if (velocidadeEsquerda > 0) {
          velocidadeEsquerda = vel_min_arduino;
        }
        else if (velocidadeEsquerda < 0){
          velocidadeEsquerda = -vel_min_arduino;
        }
        else {
          velocidadeEsquerda = 0; 
        }
      }
      else if (abs_dir < abs_esq ) {
        if (abs_dir != 0) {
          if (velocidadeEsquerda >= 0) {
            velocidadeEsquerda = (abs_esq/abs_dir)*vel_min_arduino;
          }
          else {
            velocidadeEsquerda = -(abs_esq/abs_dir)*vel_min_arduino;
          }
        }
        else {
          velocidadeEsquerda = vel_min_arduino;
        }
        if (velocidadeDireita > 0) {
          velocidadeDireita = vel_min_arduino;
        }
        else if (velocidadeDireita < 0){
          velocidadeDireita = -vel_min_arduino;
        }
        else {
          velocidadeDireita = 0;
        }
      } 
      else {
        if (velocidadeEsquerda > 0) {
          velocidadeEsquerda = vel_min_arduino;
        }
        else if (velocidadeEsquerda < 0) {
          velocidadeEsquerda = -vel_min_arduino;
        }
        else {
          velocidadeEsquerda = 0;
        }
        if (velocidadeDireita > 0) {
          velocidadeDireita = vel_min_arduino;
        }
        else if (velocidadeDireita < 0) {
          velocidadeDireita = -vel_min_arduino;
        }
        else {
          velocidadeDireita = 0;
        }
      } 
    }

#if defined(GAZEBO)
    velocidadeRobo.linear.x = VelocidadeRecebida.x;
    velocidadeRobo.angular.z = VelocidadeRecebida.z;
#endif

#if defined(ARDUINO) 
    velocidadeArduinoEsquerda.data = velocidadeEsquerda;
    velocidadeArduinoDireita.data = velocidadeDireita;
#endif
  }
  else {

#if defined(GAZEBO)
    velocidadeRobo.linear.x = 0;
    velocidadeRobo.angular.z = 0;
#endif

#if defined(ARDUINO) 
    velocidadeArduinoEsquerda.data = 0;
    velocidadeArduinoDireita.data = 0;
#endif  
  }

#if defined(ARQ_DEBUG)
  fprintf(arq2, "%f  %f \n", velocidadeEsquerda,velocidadeDireita);
#endif
#if defined(DEBUG)
  ROS_INFO("%f  %f \n", velocidadeEsquerda,velocidadeDireita);
#endif

}
void verificaObstaculosZVD (tf::TransformListener &tfListener) {

  ultrassons us1;
  std::vector<ultrassons> US_aux;
  float aux_x = 0, aux_y = 0;
  float theta = 0;
  int i;
  float f_aux = 0;
  float sx = 0, sy = 0;
  static float delta[NUM_US] = {0,0,0,0,0,0,0,0,0,0,0}, delta_atual[NUM_US] = {0,0,0,0,0,0,0,0,0,0,0};

  std::string usNames[] = {"/ultrasound1","/ultrasound2","/ultrasound3","/ultrasound4","/ultrasound5",
                                        "/ultrasound6","/ultrasound7","/ultrasound8","/ultrasound9","/ultrasound10",
                                        "/ultrasound11"};

  i = 0;
  for (i = 0; i < NUM_US; ++i){
       
    tf::StampedTransform transform;
    
    try{
      us1.id = i;
      tfListener.lookupTransform(usNames[i], "/base_link",  
      ros::Time(0), transform);
      tf::Vector3 us(US[i],0,0);
      tf::Vector3 usT = transform.inverse()(us);
      us1.x = usT.x();
      us1.y = usT.y();
      us1.z = usT.z();
      US_aux.push_back(us1);

/*#if defined(DEBUG)
      ROS_INFO("%f,%f,%f",usT.x(),usT.y(),usT.z());
#endif*/

    }catch (tf::TransformException & ex){
      ROS_ERROR("%s",ex.what());
    
    }
/*#if defined(DEBUG)
    ROS_INFO("US[%d] = %f",i+1,US[i]);
#endif*/
  
  }

  i = 0;
  f_aux = 0;
  aux_x = 0;
  aux_y = 0;

  for (i = 0; i < NUM_US; i++){
    
    if (!US_aux.empty()) {
      us1 = US_aux.front();
      US_aux.erase(US_aux.begin());  
    
      us1.x = us1.x + DISTANCIA_CENTRO;
            
      if (US[i] > DIST_MAX[i]){
        delta_atual[i] = 0;
      }
      else {
        delta_atual[i] = DIST_MAX[i] - US[i];
      }
    
      if (delta_atual[i] == 0) {//(delta_atual[i] - delta [i]) <= 0){
        f_aux += 0;
      }
      else {
        f_aux += delta_atual[i] - delta[i];
        aux_x += (SX[i] - us1.x );
        aux_y += (SY[i] - us1.y);
      }

      delta[i] = delta_atual[i];
    }
  }

  if (f_aux > 0) {
    obstaculo = true;
    fk = f_aux;
    thetak = atan2(aux_y,aux_x);
  }
  else {
    if (obstaculo){
      desviou = true;
      roboInicial = roboAtual;
    }
    fk = 0;
    thetak = 0;
    obstaculo = false;

/*
#if defined(DEBUG)
    ROS_INFO("Ultrassom %d: %f", i+1,US[i]);
#endif*/
  }

#if defined(DEBUG)
  ROS_INFO("fk: %f thetak: %f aux_x: %f aux_y: %f", fk, thetak,aux_x,aux_y);
#endif
  
}


int main(int argc, char **argv)
{

#if defined(ARQ_DEBUG)
  arq = fopen("/home/pi/Documents/feedback.txt", "w");
  arq2 = fopen("/home/pi/Documents/feedbackvel.txt", "w");
#endif

  ros::init(argc, argv, "robo_virtual");

  ros::NodeHandle n;
 
  ros::Subscriber subEnable = n.subscribe("enable_follow_path", 1000, enablePathCallback);
  ros::Subscriber subAtual = n.subscribe("odom", 1000, posicaoAtualCallback);
  ros::Subscriber subTrajeto = n.subscribe("path_planned", 1000, trajetoCallback);
  ros::Subscriber subVelocidade = n.subscribe("velocity", 1000, velocidadeCallback);

  ros::Subscriber subUS1 = n.subscribe("ultrasound1",1000,UltrassomCallback);
  ros::Subscriber subUS2 = n.subscribe("ultrasound2",1000,UltrassomCallback);
  ros::Subscriber subUS3 = n.subscribe("ultrasound3",1000,UltrassomCallback);
  ros::Subscriber subUS4 = n.subscribe("ultrasound4",1000,UltrassomCallback);
  ros::Subscriber subUS5 = n.subscribe("ultrasound5",1000,UltrassomCallback);
  ros::Subscriber subUS6 = n.subscribe("ultrasound6",1000,UltrassomCallback);
  ros::Subscriber subUS7 = n.subscribe("ultrasound7",1000,UltrassomCallback);
  ros::Subscriber subUS8 = n.subscribe("ultrasound8",1000,UltrassomCallback);
  ros::Subscriber subUS9 = n.subscribe("ultrasound9",1000,UltrassomCallback);
  ros::Subscriber subUS10 = n.subscribe("ultrasound10",1000,UltrassomCallback);
  ros::Subscriber subUS11 = n.subscribe("ultrasound11",1000,UltrassomCallback);

  tf::TransformListener tfListener;

#if defined(GAZEBO)
  ros::Publisher pubVelocidade = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
#endif

#if defined(ARDUINO)
  ros::Publisher pubVelocidadear = n.advertise<raspberry_msgs::StampedFloat32>("raspberry_float32", 1000);
#endif

  ros::Timer temporizador = n.createTimer(ros::Duration(TEMPO_AMOSTRAGEM),RoboReferencia, false);

  ros::Rate loop_rate(20);
  
#if defined(ARDUINO)
  velocidadeArduinoDireita.id = 10;
  velocidadeArduinoEsquerda.id = 11;
#endif

  float tempo = 0;

  while (ros::ok()){

    calculaSegmento();

#if defined(TESTE_US)
    controladorTrajetoria();
    verificaObstaculosZVD(tfListener); 
#endif
    //Se estiver no modo seguir trajetoria
    if (enable == SEGUIR_TRAJETORIA){
      controladorTrajetoria();
      verificaObstaculosZVD(tfListener);  
    }
    else if (enable == SEGUIR_VELOCIDADE){
      controladorVelocidade();
    }
    else if (enable == APROXIMAR_CONE){
      controladorTrajetoria();
    }
#if defined (TESTE_US)
    else if (enable == PARA && !obstaculo) {
      parar = true;
      inicio = false;
      erro1 = 0, erro2 = 0, erro3 = 0;
#if defined(ARDUINO)
      velocidadeArduinoEsquerda.data = 0;
      velocidadeArduinoDireita.data = 0;
#endif
#if defined(GAZEBO)
      velocidadeRobo.linear.x = 0;
      velocidadeRobo.angular.z = 0;
#endif
    }

#else 
    else if (enable == PARA && !obstaculo) {
      parar = true;
      inicio = false;
      erro1 = 0, erro2 = 0, erro3 = 0;
#if defined(ARDUINO)
      velocidadeArduinoEsquerda.data = 0;
      velocidadeArduinoDireita.data = 0;
#endif
#if defined(GAZEBO)
      velocidadeRobo.linear.x = 0;
      velocidadeRobo.angular.z = 0;
#endif
    }
#endif


#if defined(GAZEBO)
    pubVelocidade.publish(velocidadeRobo);
#endif

#if defined(ARDUINO)
    pubVelocidadear.publish(velocidadeArduinoDireita);
    pubVelocidadear.publish(velocidadeArduinoEsquerda);
#endif

#if defined(DEBUG)
    #warning Atencao! DEBUG esta definido!
    ROS_INFO("%f %f %f %f %f %f %f",tempo, roboReferencia.x, roboReferencia.y, roboReferencia.theta,roboAtual.x, roboAtual.y, roboAtual.theta);
#endif
#if defined(ARQ_DEBUG)
    fprintf(arq,"%f %f %f %f %f %f %f\n",tempo, roboReferencia.x, roboReferencia.y, roboReferencia.theta,roboAtual.x, roboAtual.y, roboAtual.theta); 
#endif
    loop_rate.sleep();
    ros::spinOnce();
  
  }
#if defined(ARQ_DEBUG)
  fclose(arq);
#endif
  return 0;
}
