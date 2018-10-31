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

Vector3d gps[NUM_ITERATE];
Vector6d msf_state[200];
Vector3d acc[200];
double dtime[200];
int Iterate = 0;
Vector6d state;
int Numimu = 0;
int Numstate = 0;
void imucallback(const sensor_msgs::Imu::ConstPtr &data)
{
  // imu = *data;
  sensor_msgs::Imu imu_bf = *data;

  static ros::Time st = ros::Time::now();
  static ros::Duration ct = ros::Time::now()-st;
  static ros::Duration pt= ct;
  ct = ros::Time::now()-st;
  dtime[Numimu] = (ct - pt).toSec();

  //transform bf to ef
  tf::Quaternion qimu(imu_bf.linear_acceleration.x,imu_bf.linear_acceleration.y,imu_bf.linear_acceleration.z,0);
  tf::Quaternion q(imu_bf.orientation.x,imu_bf.orientation.y,imu_bf.orientation.z,imu_bf.orientation.w);

  qimu = q*qimu*q.inverse() - tf::Quaternion(0,0,9.81,0);

  imu.linear_acceleration.x = qimu.x();
  imu.linear_acceleration.y = qimu.y();
  imu.linear_acceleration.z = qimu.z();
  imu.orientation = imu_bf.orientation;

  acc[Numimu][0] = imu.linear_acceleration.x;
  acc[Numimu][1] = imu.linear_acceleration.y;
  acc[Numimu][2] = imu.linear_acceleration.z;

  Numimu++;
  // tf::Quaternion q(imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w);
  // tf::Matrix3x3 m(q);
  // m.getRPY(roll, pitch, yaw);
  //ROS_INFO("ax:[%f]", yaw);
}
void gpscallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& data) {
  gps_pose = *data;

  if(Iterate>=NUM_ITERATE) return;

  gps[Iterate][0] = gps_pose.pose.pose.position.x;
  gps[Iterate][1] = gps_pose.pose.pose.position.y;
  gps[Iterate][2] = gps_pose.pose.pose.position.z;

  Iterate++;

}
void state_out_callback(const sensor_fusion_comm::DoubleArrayStamped::ConstPtr& data) {

  state_out_msf = *data;

  for(int i =0;i<6;i++)
    msf_state[Numstate][i] = state_out_msf.data[i];
  Numstate++;
}

void Optimize_test(double dt);


int main( int argc, char** argv )
{
  ros::init(argc, argv, "gps_tester");
  ros::NodeHandle n;
  ros::Rate r(200);

  ros::Subscriber imu_sub           = n.subscribe<sensor_msgs::Imu>("/imu_max", 2, imucallback);
  ros::Subscriber gps_sub           = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/imu_max/pose", 2, gpscallback);
  ros::Subscriber state_out_msf_sub = n.subscribe<sensor_fusion_comm::DoubleArrayStamped>("/msf_core/state_out", 2, state_out_callback);
  ros::Publisher  poseop_pub        = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/optimized", 10);

  static ros::Time start_time = ros::Time::now();
  static ros::Duration cur_time = ros::Time::now()-start_time;
  static ros::Duration prev_time= cur_time;
  static double dt = 0.1;  

  poseop_msgs.header.frame_id = "odom";

  state_out_msf.data.resize(36);
  // state.setZero();
  // Iterate over the simulation steps


  state.setZero();
  for (int k = 0; k < 3; k++)
    {
      state[k] = 0;
    }
  while(ros::ok())
    {

      // saytab("Iter ");sayend(Iterate);
      // static int k = 1;
      ros::spinOnce();
      r.sleep();
      cur_time = ros::Time::now()-start_time;
      dt = (cur_time - prev_time).toSec();
      if(Iterate >= NUM_ITERATE) {
        prev_time = cur_time;
        sayend("START");
        Optimize_test(dt);
        sayend("END");
        Iterate=0;
        Numimu=0;
        Numstate = 0;
        poseop_pub.publish(poseop_msgs);
      }
      
      // saytab("cur="); saytab(cur_time.toSec());
      // saytab("prev=");saytab(prev_time.toSec());
      // saytab("dt=");  sayend(dt);
    }
}



// void Optimize_test(double dt ) {
//     // Set up the optimiser
//   SparseOptimizer optimizer;
//   optimizer.setVerbose(false);

//   // Create the block solver - the dimensions are specified because
//   // 3D observations marginalise to a 3D estimate
//   typedef BlockSolver<BlockSolverTraits<3, 3> > BlockSolver_3_3;
//   BlockSolver_3_3::LinearSolverType* linearSolver
//       = new LinearSolverCholmod<BlockSolver_3_3::PoseMatrixType>();
//   BlockSolver_3_3* blockSolver
//       = new BlockSolver_3_3(linearSolver);
//   OptimizationAlgorithmGaussNewton* solver
//     = new OptimizationAlgorithmGaussNewton(blockSolver);
//   optimizer.setAlgorithm(solver);

//   // Sample the actual location of the target
//   Vector3d truePoint(0,
//                      0,
//                      0);


//         // Construct vertex which corresponds to the actual point of the target
//   VertexPosition3D* position = new VertexPosition3D();
//   position->setId(0);
//   optimizer.addVertex(position);


//   // Now generate some noise corrupted measurements; for simplicity
//   // these are uniformly distributed about the true target. These are
//   // modelled as a unary edge because they do not like to, say,
//   // another node in the map.
//   int numMeasurements = Iterate;
//   double noiseLimit = sqrt(12.);
//   double noiseSigma = noiseLimit*noiseLimit / 12.0;
//   double noiseSigma2 = noiseSigma;

//   for (int i = 0; i < numMeasurements; i++)
//     {
//         for (int k = 0; k < Numstate; k++)
//           {
//             saytab("stage2");
//             Vector3d measurement2 = msf_state[k];
//             GPSObservationPosition3DEdge* goe2 = new GPSObservationPosition3DEdge();
//             goe2->setVertex(0, position);
//             goe2->setMeasurement(measurement2);
//             goe2->setInformation(Matrix3d::Identity() / noiseSigma2);
//             optimizer.addEdge(goe2);
//             saytab("stage3");


//           }
//       lastposition = position;

//       Vector3d measurement =gps[i];
//       GPSObservationPosition3DEdge* goe = new GPSObservationPosition3DEdge();
//       goe->setVertex(0, position);
//       goe->setMeasurement(measurement);
//       goe->setInformation(Matrix3d::Identity() / noiseSigma);
//       optimizer.addEdge(goe);


//               // Construct vertex which corresponds to the actual point of the target
//     VertexPosition3D* position = new VertexPosition3D();
//     position->setId(i+1);
//     optimizer.addVertex(position);
//     }

//   // Configure and set things going
//   optimizer.initializeOptimization();
//   optimizer.setVerbose(true);
//   optimizer.optimize(5);
  
//   cout << "truePoint=\n" << truePoint << endl;

//   cerr <<  "computed estimate=\n" << dynamic_cast<VertexPosition3D*>(optimizer.vertices().find(0)->second)->estimate() << endl;

//   //position->setMarginalized(true);
  
//   SparseBlockMatrix<MatrixXd> spinv;

//   optimizer.computeMarginals(spinv, position);



//   //optimizer.solver()->computeMarginals();

//   // covariance
//   //
//   cout << "covariance\n" << spinv << endl;

//   cout << spinv.block(0,0) << endl;



//   Vector3d v =  dynamic_cast<VertexPosition3D*>(optimizer.vertices().find(0)->second)->estimate();



//        poseop_msgs.header.stamp = ros::Time::now();
//        poseop_msgs.pose.pose.position.x = v[0];
//        poseop_msgs.pose.pose.position.y = v[1];
//        poseop_msgs.pose.pose.position.z = v[2];
       
//        poseop_msgs.pose.pose.orientation = imu.orientation;

//        sayend("CAL COV");
//       // SparseBlockMatrix<MatrixXd> spinv;

//       // optimizer.computeMarginals(spinv, position);



//       // //optimizer.solver()->computeMarginals();

//       // // covariance
//       // //
//       // cout << "covariance\n" << spinv << endl;

//       // for(int u =0;u<6;u++)
//       //   for(int v=0;v<6;v++)
//       //     poseop_msgs.pose.covariance[6*u+v] = spinv(u, v);
//       poseop_msgs.pose.covariance[0]=0.1;
//       poseop_msgs.pose.covariance[7]=0.1;
//       poseop_msgs.pose.covariance[14]=0.1;
//       poseop_msgs.pose.covariance[21]=0.1;
//       poseop_msgs.pose.covariance[28]=0.1;
//       poseop_msgs.pose.covariance[35]=0.1;
// }


















void Optimize_test(double dt) {
 // Set up the parameters of the simulation
  int numberOfTimeSteps = NUM_ITERATE;
  const double processNoiseSigma = 0.05;
  const double accelerometerNoiseSigma = 0.038;
  const double gpsNoiseSigma = 1;
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
  VertexPositionVelocity3D* stateNode = new VertexPositionVelocity3D();
  stateNode->setEstimate(state);
  stateNode->setId(0);
  optimizer.addVertex(stateNode);

  // Set up last estimate
  VertexPositionVelocity3D* lastStateNode = stateNode;

  // Iterate over the simulation steps
  for (int k = 0; k < numberOfTimeSteps; ++k)
    {   
      for(int j = 0; j < Numstate-1; j++) 
      {
        saytab(j);saytab(Numstate);saytab(Numimu);sayend(Iterate);

          state = msf_state[j];
          // Construct vertex which corresponds to the current state of the target
          VertexPositionVelocity3D* stateNode = new VertexPositionVelocity3D();
          
          stateNode->setId((Numstate-1)*k+j+1);
          stateNode->setEstimate(state);
          stateNode->setMarginalized(false);
          optimizer.addVertex(stateNode);

          sayend("STEP1");
          // Construct the accelerometer measurement
          Vector3d accelerometerMeasurement = acc[j];

          TargetOdometry3DEdge* toe = new TargetOdometry3DEdge(0.01, accelerometerNoiseSigma);
          toe->setVertex(0, lastStateNode);
          toe->setVertex(1, stateNode);
          VertexPositionVelocity3D* vPrev= dynamic_cast<VertexPositionVelocity3D*>(lastStateNode);
          VertexPositionVelocity3D* vCurr= dynamic_cast<VertexPositionVelocity3D*>(stateNode);
          toe->setMeasurement(accelerometerMeasurement);
          optimizer.addEdge(toe);
          sayend("STEP2");
          // compute the initial guess via the odometry
          g2o::OptimizableGraph::VertexSet vPrevSet;
          vPrevSet.insert(vPrev);
          toe->initialEstimate(vPrevSet,vCurr);

          lastStateNode = stateNode;
          sayend("STEP3");
      }
      
        // Construct the GPS observation
        Vector3d gpsMeasurement = gps[k];

      sayend("STEP21");
      // Add the GPS observation
      GPSObservationEdgePositionVelocity3D* goe = new GPSObservationEdgePositionVelocity3D(gpsMeasurement, gpsNoiseSigma);
      goe->setVertex(0, stateNode);
      optimizer.addEdge(goe);
      sayend("STEP22");
    }

  // Configure and set things going
  optimizer.initializeOptimization();
  optimizer.setVerbose(true);
  optimizer.optimize(5);
  cerr << "number of vertices:" << optimizer.vertices().size() << endl;
  cerr << "number of edges:" << optimizer.edges().size() << endl;

  // Print the results

  cout << "state=\n" << state << endl;

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

       Vector6d v = dynamic_cast<VertexPositionVelocity3D*>(optimizer.vertices().find((std::max)(numberOfTimeSteps-1,0))->second)->estimate();


       poseop_msgs.header.stamp = ros::Time::now();
       poseop_msgs.pose.pose.position.x = v[0];
       poseop_msgs.pose.pose.position.y = v[1];
       poseop_msgs.pose.pose.position.z = v[2];
       
       poseop_msgs.pose.pose.orientation = imu.orientation;


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







