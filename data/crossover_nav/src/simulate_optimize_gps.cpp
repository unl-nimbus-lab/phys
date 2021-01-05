// Prepare for G2O-------------------------------------------------

#include <Eigen/StdVector>
#include <iostream>

#include <stdint.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/stuff/sampler.h>

#include "targetTypes6D.hpp"
// #include "continuous_to_discrete.h"
// #include "targetTypes3D.hpp"
using namespace Eigen;
using namespace std;
using namespace g2o;



// Prepare for ROS-------------------------------------------------
#include <ros/ros.h>
#include <ros/time.h>
#include "sensor_msgs/Imu.h"
// #include "sensor_msgs/MagneticField.h"
// #include "sensor_msgs/Temperature.h"
// #include "sensor_msgs/FluidPressure.h"
// #include "sensor_msgs/Joy.h"
// #include "sensor_msgs/NavSatFix.h"
// #include "sensor_msgs/Range.h"

// #include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
// #include "geometry_msgs/PoseStamped.h"
// #include "geometry_msgs/TwistWithCovarianceStamped.h"
// #include <geometry_msgs/Vector3Stamped.h>
// #include <sensor_fusion_comm/ExtEkf.h>
#include <sensor_fusion_comm/DoubleArrayStamped.h>

// #include "nav_msgs/Odometry.h"
// #include "crossover_nav/Ack.h"
// #include "crossover_nav/Navdata.h"
// #include "crossover_nav/odom_data.h"
// #include "crossover_nav/Status.h"

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>




// #include <dynamic_reconfigure/BoolParameter.h>
// #include <dynamic_reconfigure/Reconfigure.h>
// #include <dynamic_reconfigure/Config.h>

// #include <termios.h>

//function convert arduino to ros
//------------ROS version-------
// #include <iostream>
inline long millis() {
  return (1e3 * ros::Time::now().sec + 1e-6 * ros::Time::now().nsec);
}
inline long micros() {
  return (1e6 * ros::Time::now().sec + 1e-3 * ros::Time::now().nsec);
}
//not use millis() it give me online floating point behind sec
//go use ros::Time::now();
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define delay(x) ros::Duration(x/1000.0).sleep()
#define say(x) (std::cout << x)
#define sayend(x) (std::cout << x << std::endl)
#define saytab(x) (std::cout << x << "\t")
#define saycomma(x) (std::cout << x << " ,")
#define Min(a,b) ((a)<(b)?(a):(b))
#define Max(a,b) ((a)>(b)?(a):(b))
#define isfinite(X) std::isfinite(X)
#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))
#define wrap_pi(x) (x < -3.14 ? x+6.28 : (x > 3.14 ? x - 6.28: x))
#define HALF_M_PI 1.570796

sensor_msgs::Imu imu;
geometry_msgs::PoseWithCovarianceStamped gps_pose , poseop_msgs;
sensor_fusion_comm::DoubleArrayStamped state_out_msf;
#define NUM_ITERATE 1

Vector3d gps[5];
Vector6d msf_state[17];
Vector3d acc[17];
double dtime[200];
int Iterate = 0;
Vector6d state;
int Numimu = 0;
int Numstate = 0;

int num_state = 0; //round max = 17
float GPS_lOOCKUP[5][2] = {{0,0},
                         {3,0},
                         {3,-3},
                         {0,-3},
                         {0,0}
                          };

float STATE_LOOCKUP[17][2];








// void imucallback(const sensor_msgs::Imu::ConstPtr &data)
// {
//   // imu = *data;
//   sensor_msgs::Imu imu_bf = *data;

//   static ros::Time st = ros::Time::now();
//   static ros::Duration ct = ros::Time::now()-st;
//   static ros::Duration pt= ct;
//   ct = ros::Time::now()-st;
//   dtime[Numimu] = (ct - pt).toSec();

//   //transform bf to ef
//   tf::Quaternion qimu(imu_bf.linear_acceleration.x,imu_bf.linear_acceleration.y,imu_bf.linear_acceleration.z,0);
//   tf::Quaternion q(imu_bf.orientation.x,imu_bf.orientation.y,imu_bf.orientation.z,imu_bf.orientation.w);

//   qimu = q*qimu*q.inverse() - tf::Quaternion(0,0,9.81,0);

//   imu.linear_acceleration.x = qimu.x();
//   imu.linear_acceleration.y = qimu.y();
//   imu.linear_acceleration.z = qimu.z();
//   imu.orientation = imu_bf.orientation;

//   acc[Numimu][0] = imu.linear_acceleration.x;
//   acc[Numimu][1] = imu.linear_acceleration.y;
//   acc[Numimu][2] = imu.linear_acceleration.z;

//   Numimu++;
//   // tf::Quaternion q(imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w);
//   // tf::Matrix3x3 m(q);
//   // m.getRPY(roll, pitch, yaw);
//   //ROS_INFO("ax:[%f]", yaw);
// }
// void gpscallback(/*const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& data*/) {
//   // gps_pose = *data;

//   if(Iterate>=NUM_ITERATE) return;

//   gps[num_state][0] = GPS_lOOCKUP[0][0];
//   gps[num_state][1] = gps_pose.pose.pose.position.y;
//   gps[num_state][2] = 0;

//   Iterate++;

// }
// void state_out_callback(/*const sensor_fusion_comm::DoubleArrayStamped::ConstPtr& data*/) {

//   state_out_msf = *data;

//   for(int i =0;i<6;i++)
//     msf_state[Numstate][i] = state_out_msf.data[i];
//   Numstate++;
// }

void Optimize_test(double dt);


int main( int argc, char** argv )
{
  ros::init(argc, argv, "gps_tester");
  ros::NodeHandle n;
  ros::Rate r(200);

  // ros::Subscriber imu_sub           = n.subscribe<sensor_msgs::Imu>("/imu_max", 2, imucallback);
  // ros::Subscriber gps_sub           = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/imu_max/pose", 2, gpscallback);
  // ros::Subscriber state_out_msf_sub = n.subscribe<sensor_fusion_comm::DoubleArrayStamped>("/msf_core/state_out", 2, state_out_callback);
  ros::Publisher  poseop_pub        = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/optimized", 10);

  static ros::Time start_time = ros::Time::now();
  static ros::Duration cur_time = ros::Time::now()-start_time;
  static ros::Duration prev_time= cur_time;
  static double dt = 0.25;  

  poseop_msgs.header.frame_id = "odom";

  state_out_msf.data.resize(36);
  // state.setZero();
  // Iterate over the simulation steps


  state.setZero();
  for (int k = 0; k < 3; k++)
    {
      state[k] = 0;
    }



  STATE_LOOCKUP[0][0] = 0;
  STATE_LOOCKUP[0][1] = 0;

  for(int i=1;i<=4;i++) {
    /*x*/STATE_LOOCKUP[i][0] = (3.0/4.0)*i;
    /*y*/STATE_LOOCKUP[i][1] = 0;
    if(i==1 || i==4) {
      acc[i] = {16*3/3,0,0};
      if(i==4) acc[i]*=-1;
    }else
      acc[i] = {0,0,0};
  }
  for(int i=1;i<=4;i++) {
    /*x*/STATE_LOOCKUP[i+4][0] = 3.0+(1.0/4.0)*i;
    /*y*/STATE_LOOCKUP[i+4][1] = -i;
    if(i==1 || i==4) {
      acc[i+4] = {16*1/3,-16*4/3,0};
      if(i==4) acc[i+4]*=-1;
    }else
      acc[i+4] = {0,0,0};
  }
  for(int i=1;i<=4;i++) {
    /*x*/STATE_LOOCKUP[i+8][0] = 4.0+ (-3.0/4.0)*i;
    /*y*/STATE_LOOCKUP[i+8][1] = -4.0 + (1.0/4.0)*i;
    if(i==1 || i==4) {
      acc[i+8] = {-16*3/3,16*1/3,0};
      if(i==4) acc[i+8]*=-1;
    }else
      acc[i+8] = {0,0,0};
  }
  for(int i=1;i<=4;i++) {
    /*x*/STATE_LOOCKUP[i+12][0] = 1.0;
    /*y*/STATE_LOOCKUP[i+12][1] = -3.0+i;
    if(i==1 || i==4) {
      acc[i+12] = {16*0/3,16*4/3,0};
      if(i==4) acc[i+12]*=-1;
    }else
      acc[i+12] = {0,0,0};
  }

  for(int i=0;i<17;i++) {
   saytab(acc[i][0]); sayend(acc[i][1]);
 }


  while(ros::ok() && Iterate <= 3)
    {

        gps[Iterate+1][0] = GPS_lOOCKUP[Iterate+1][0];
        gps[Iterate+1][1] = GPS_lOOCKUP[Iterate+1][1];
        gps[Iterate+1][2] = 0;

        for(int i = 1;i<=4;i++) {
          msf_state[Iterate*4+i][0] = STATE_LOOCKUP[Iterate*4+i][0];
          msf_state[Iterate*4+i][1] = STATE_LOOCKUP[Iterate*4+i][1];
          msf_state[Iterate*4+i][2] = 0;
          msf_state[Iterate*4+i][3] = 0;
          msf_state[Iterate*4+i][4] = 0;
          msf_state[Iterate*4+i][5] = 0;
          // Numstate++;
        }
        
        Iterate ++;
      }
        saytab("START Iterate = ");sayend(Iterate);
        Optimize_test(dt);
        sayend("END");
        
    
}



void Optimize_test(double dt) {
 // Set up the parameters of the simulation
  int numberOfTimeSteps = NUM_ITERATE;
  const double processNoiseSigma = 0.05;
  const double accelerometerNoiseSigma = 0.01;
  const double gpsNoiseSigma = 0.001;
  // const double dt = 1;  

  // Set up the optimiser and block solver
  SparseOptimizer optimizer;
  optimizer.setVerbose(false);

  typedef BlockSolver< BlockSolverTraits<6, 6> > BlockSolver;
  BlockSolver::LinearSolverType * linearSolver
      = new LinearSolverCholmod<BlockSolver::PoseMatrixType>();
  BlockSolver* blockSolver = new BlockSolver(linearSolver);
  OptimizationAlgorithm* optimizationAlgorithm = new OptimizationAlgorithmGaussNewton(blockSolver);
  optimizer.setAlgorithm(optimizationAlgorithm);

  // Sample the start location of the target
  

    state = msf_state[0];

  // Construct the first vertex; this corresponds to the initial
  // condition and register it with the optimiser
  saytab("add vertex state ");sayend(0);
  VertexPositionVelocity3D* stateNode = new VertexPositionVelocity3D();
  stateNode->setEstimate(state);
  stateNode->setId(0);
  optimizer.addVertex(stateNode);

  // Construct the GPS observation
  Vector3d gpsMeasurement = gps[0];
  saytab("add edge gps ");sayend(0);
  // Add the GPS observation
  GPSObservationEdgePositionVelocity3D* goe = new GPSObservationEdgePositionVelocity3D(gpsMeasurement, gpsNoiseSigma);
  goe->setMeasurement(gpsMeasurement);
  goe->setVertex(0, stateNode);
  optimizer.addEdge(goe);

  // Construct the msf observation
  Vector3d msfMeasurement(0,0,0);

  saytab("add edge statemsf ");sayend(0);
  // Add the msf observation
  GPSObservationEdgePositionVelocity3D* goe2 = new GPSObservationEdgePositionVelocity3D(msfMeasurement, 0.1);
  goe2->setMeasurement(msfMeasurement);
  goe2->setVertex(0, stateNode);
  optimizer.addEdge(goe2);


  // Set up last estimate
  VertexPositionVelocity3D* lastStateNode = stateNode;

  // Iterate over the simulation steps
  for (int k = 0; k < Iterate; ++k)
    {   
      for(int j = 1; j <= 4/*Numstate per round*/; j++) 
      {
        // saytab(j);saytab(Numstate);sayend(Iterate);

          // state = msf_state[k*4+j];
        

          // Construct vertex which corresponds to the current state of the target
          VertexPositionVelocity3D* stateNode = new VertexPositionVelocity3D();
          
          stateNode->setId(k*4+j);
          stateNode->setEstimate(state);
          stateNode->setMarginalized(false);
          optimizer.addVertex(stateNode);

          saytab("add vertex state ");sayend(k*4+j);












          // // Construct the accelerometer measurement
          Vector3d accelerometerMeasurement/*(msf_state[k*4+j][0],msf_state[k*4+j][1],0);//*/ = acc[k*4+j];

          TargetOdometry3DEdge* toe = new TargetOdometry3DEdge(dt, accelerometerNoiseSigma);
          toe->setVertex(0, lastStateNode);
          toe->setVertex(1, stateNode);
          VertexPositionVelocity3D* vPrev= dynamic_cast<VertexPositionVelocity3D*>(lastStateNode);
          VertexPositionVelocity3D* vCurr= dynamic_cast<VertexPositionVelocity3D*>(stateNode);
          toe->setMeasurement(accelerometerMeasurement);
          optimizer.addEdge(toe);
          // compute the initial guess via the odometry
          g2o::OptimizableGraph::VertexSet vPrevSet;
          vPrevSet.insert(vPrev);
          toe->initialEstimate(vPrevSet,vCurr);

          lastStateNode = stateNode;
      }
                // Construct the msf observation
          Vector3d gpsMeasurement2 = gps[k+1];

          // Add the msf observation
          GPSObservationEdgePositionVelocity3D* goe2 = new GPSObservationEdgePositionVelocity3D(gpsMeasurement2, 0.1);
          goe2->setMeasurement(gpsMeasurement2);
          goe2->setVertex(0, stateNode);
          optimizer.addEdge(goe2);


        // Construct the GPS observation
        Vector3d gpsMeasurement = gps[k+1];

      saytab("add edge gps ");sayend(k+1);
      // Add the GPS observation
      GPSObservationEdgePositionVelocity3D* goe = new GPSObservationEdgePositionVelocity3D(gpsMeasurement, gpsNoiseSigma);
      goe->setMeasurement(gpsMeasurement);
      goe->setVertex(0, stateNode);
      optimizer.addEdge(goe);
    }

  // Configure and set things going
  optimizer.initializeOptimization();
  optimizer.setVerbose(true);
  optimizer.optimize(5);
  cerr << "number of vertices:" << optimizer.vertices().size() << endl;
  cerr << "number of edges:" << optimizer.edges().size() << endl;

  // Print the results

  // cout << "state=\n" << state << endl;

#if 0
  for (int k = 0; k < numberOfTimeSteps; k++)
    {
      cout << "computed estimate " << k << "\n"
           << dynamic_cast<VertexPositionVelocity3D*>(optimizer.vertices().find(k)->second)->estimate() << endl;
       }
#endif

  // Vector6d v1 = dynamic_cast<VertexPositionVelocity3D*>(optimizer.vertices().find((std::max)(numberOfTimeSteps-2,0))->second)->estimate();
  // Vector6d v2 = dynamic_cast<VertexPositionVelocity3D*>(optimizer.vertices().find((std::max)(numberOfTimeSteps-1,0))->second)->estimate();
  // cout << "v1=\n" << v1 << endl;
  // cout << "v2=\n" << v2 << endl;
  // cout << "delta state=\n" << v2-v1 << endl;
       // Vector6d v[4];
       Vector6d v[17];
       for(int i=0;i<17;i++)
          v[i] = dynamic_cast<VertexPositionVelocity3D*>(optimizer.vertices().find(i)->second)->estimate();
      for(int i=0;i<17;i++)
       cout << v[i][0] << "\t" << v[i][1] <<endl;

      cout << endl;
      cout << endl;
      cout << endl;
      for(int i=0;i<17;i++)
       cout << msf_state[i][0] << "\t" << msf_state[i][1] <<endl;

      cout << endl;
      cout << endl;
      cout << endl;
      for(int i=0;i<5;i++)
        cout << gps[i][0] << "\t" << gps[i][1] <<endl;
       // poseop_msgs.header.stamp = ros::Time::now();
       // poseop_msgs.pose.pose.position.x = v[0];
       // poseop_msgs.pose.pose.position.y = v[1];
       // poseop_msgs.pose.pose.position.z = v[2];
       
       // poseop_msgs.pose.pose.orientation = imu.orientation;


      SparseBlockMatrix<MatrixXd> spinv;

      optimizer.computeMarginals(spinv, stateNode);



      //optimizer.solver()->computeMarginals();

      // covariance
      //
      cout << "covariance\n" << spinv << endl;

      // for(int u =0;u<6;u++)
      //   for(int v=0;v<6;v++)
      //     poseop_msgs.pose.covariance[6*u+v] = spinv(u, v);
      poseop_msgs.pose.covariance[0]=0.1;
      poseop_msgs.pose.covariance[7]=0.1;
      poseop_msgs.pose.covariance[14]=0.1;
      poseop_msgs.pose.covariance[21]=0.1;
      poseop_msgs.pose.covariance[28]=0.1;
      poseop_msgs.pose.covariance[35]=0.1;
      
}









// #ifndef G2O_TARGET_TYPES_6D_HPP_
// #define G2O_TARGET_TYPES_6D_HPP_

// #include <g2o/core/base_vertex.h>
// #include <g2o/core/base_binary_edge.h>
// #include <g2o/core/base_unary_edge.h>
// #include <Eigen/Core>

// using namespace g2o;

// typedef Eigen::Matrix<double,6,1> Vector6d;
// typedef Eigen::Matrix<double,6,6> Matrix6d;

// // This header file specifies a set of types for the different
// // tracking examples; note that 

// class VertexPosition3D : public g2o::BaseVertex<3, Eigen::Vector3d>
// {
// public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//   VertexPosition3D()
//   {
//   }
  
//   virtual void setToOriginImpl() {
//     _estimate.setZero();
//   }
  
//   virtual void oplusImpl(const double* update)
//   {
//     _estimate[0] += update[0];
//     _estimate[1] += update[1];
//     _estimate[2] += update[2];
//   }
  
//   virtual bool read(std::istream& /*is*/)
//   {
//     return false;
//   }
  
//   virtual bool write(std::ostream& /*os*/) const
//   {
//     return false;
//   }
  
// };

// class PositionVelocity3DEdge
// {
// };
 
// class VertexPositionVelocity3D : public g2o::BaseVertex<6, Vector6d>
// {
// public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//   VertexPositionVelocity3D()
//   {
//   }
  
//   virtual void setToOriginImpl() {
//     _estimate.setZero();
//   }
  
//   virtual void oplusImpl(const double* update)
//   {
//     for (int k = 0; k < 6; k++)
//       _estimate[k] += update[k];
//   }
  

//   virtual bool read(std::istream& /*is*/)
//   {
//     return false;
//   }
  
//   virtual bool write(std::ostream& /*os*/) const
//   {
//     return false;
//   }
  
// };

// // The odometry which links pairs of nodes together
// class TargetOdometry3DEdge : public g2o::BaseBinaryEdge<6, Eigen::Vector3d, VertexPositionVelocity3D, VertexPositionVelocity3D>
// {
// public:
//   TargetOdometry3DEdge(double dt, double noiseSigma)
//   {
//     _dt = dt;

//     double q = noiseSigma * noiseSigma;
//     double dt2 = dt * dt;

//     // Process noise covariance matrix; this assumes an "impulse"
//     // noise model; we add a small stabilising term on the diagonal to make it invertible
//     Matrix6d Q=Matrix6d::Zero();
//     Q(0, 0) = Q(1,1) = Q(2,2) = dt2*dt2*q/4 + 1e-4;
//     Q(0, 3) = Q(1, 4) = Q(2, 5) = dt*dt2*q/2;
//     Q(3, 3) = Q(4,4) = Q(5,5) = dt2 * q + 1e-4;
//     Q(3, 0) = Q(4, 1) = Q(5, 2) = dt*dt2*q/2;

//     setInformation(Q.inverse());
//   }

//   /** set the estimate of the to vertex, based on the estimate of the from vertex in the edge. */
//   virtual void initialEstimate(const g2o::OptimizableGraph::VertexSet& from, g2o::OptimizableGraph::Vertex* to){
//     assert(from.size() == 1);
//     const VertexPositionVelocity3D* vi = static_cast<const VertexPositionVelocity3D*>(*from.begin());
//     VertexPositionVelocity3D* vj = static_cast<VertexPositionVelocity3D*>(to);
//     Vector6d viEst=vi->estimate();
//     Vector6d vjEst=viEst;

//     for (int m = 0; m < 3; m++)
//     {
//       vjEst[m] += _dt * (vjEst[m+3] + 0.5 * _dt * _dt* _measurement[m]);
//     }

//     for (int m = 0; m < 3; m++)
//     {
//       vjEst[m+3] =0;//+= _dt * _measurement[m];
//     }

//     vjEst[0] = _measurement[0];
//     vjEst[1] = _measurement[1];
//     vjEst[2] = 0;
//     vj->setEstimate(vjEst);
//   }

//   /** override in your class if it's not possible to initialize the vertices in certain combinations */
//   virtual double initialEstimatePossible(const g2o::OptimizableGraph::VertexSet& from, g2o::OptimizableGraph::Vertex* to) {
//     //only works on sequential vertices
//     const VertexPositionVelocity3D* vi = static_cast<const VertexPositionVelocity3D*>(*from.begin());
//     return (to->id() - vi->id() == 1) ? 1.0 : -1.0;
//   }


//   void computeError()
//   {
//     const VertexPositionVelocity3D* vi = static_cast<const VertexPositionVelocity3D*>(_vertices[0]);
//     const VertexPositionVelocity3D* vj = static_cast<const VertexPositionVelocity3D*>(_vertices[1]);
    
//     for (int k = 0; k < 3; k++)
//       {
//         _error[k] = vi->estimate()[k] /*+ _dt * (vi->estimate()[k+3] + 0.5 * _dt * _measurement[k]) - vj->estimate()[k]*/-_measurement[k];
//       }
//     for (int k = 3; k < 6; k++)
//       {
//         _error[k] = 0;//vi->estimate()[k] + _dt * _measurement[k-3]- vj->estimate()[k];
//       }
//   }
  
//   virtual bool read(std::istream& /*is*/)
//   {
//     return false;
//   }
  
//   virtual bool write(std::ostream& /*os*/) const
//   {
//     return false;
//   }

// private:
//   double _dt;
// };

// // The GPS 
// class GPSObservationEdgePositionVelocity3D : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPositionVelocity3D>
// {
// public:
//   GPSObservationEdgePositionVelocity3D(const Eigen::Vector3d& measurement, double noiseSigma)
//   {
//     setMeasurement(measurement);
//     setInformation(Eigen::Matrix3d::Identity() / (noiseSigma*noiseSigma));
//   }
  
//   void computeError()
//   {
//     const VertexPositionVelocity3D* v = static_cast<const VertexPositionVelocity3D*>(_vertices[0]);
//     for (int k = 0; k < 3; k++)
//       {
//         _error[k] = v->estimate()[k] - _measurement[k];
//       }    
//   }
  
//   virtual bool read(std::istream& /*is*/)
//   {
//     return false;
//   }
  
//   virtual bool write(std::ostream& /*os*/) const
//   {
//     return false;
//   }
// };


// #endif //  __TARGET_TYPES_6D_HPP__
