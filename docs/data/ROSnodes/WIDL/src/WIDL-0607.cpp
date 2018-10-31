#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>


#include "MATv2/Mat.h"
#include "EKF/EKF.h"
#include "HoughT.h"
#include "Quaternion.h"
#include "Grid2.h"

#include "LSONSC/LSOOThetaWIDL.h"

#define debug_lvl1
//#define debug_lvl2
#define debug_lvl3	
//#define debug_lvl4	
#define debug_lvl6	//covar estimation on delta_pose.

#define INF 1.0f/numeric_limits<float>::epsilon()
//#define PI 3.1415926535f
#define RANGE_HEUR n



#define 	maxNBRPOINT 	1200			//the incapacity to run at least at the same frame rate as the one of the sensor make the estimation flawed...
#define 	doubleMatchOnlyEstimation	//not mentioned but a choice has to be made... and it gives better results.


#define BOX

//In order to use the optimization and inferences that I've try to implement:
//#define optimDP

#define optimHAYAIinit			//in order to replace the odometry return.
#define doubleMatchesOnlyHAYAI
//#define optimHAYAIinit1
//#define optim

#define 	iterativeHAYAI
#define centroidDoubleMatchesUse		//it is to be used with iterativeHAYAI : give tremendous better results... It filters out the false positive matches.
//#define LSOO

//#define affineDeplacementAfterRotation
//#define LOWPASS_FILTER



//#define OCCUPANCY_GRID_USE
#ifdef OCCUPANCY_GRID_USE
	#define	NAVMSGSGRID
#endif

#ifdef NAVMSGSGRID
#include <nav_msgs/OccupancyGrid.h>
#endif

#define robustify

#ifdef CORRECTMOTIONMODEL
#define PROPAGATEVW?	//somehow usually wrong : the velocity of any mass isn't conserved through time due to friction with air and the ground.
						//but it give better results TODO : estimate correct covariance matrix...
template<typename T>
Mat<T> motionModel( Mat<T> state, Mat<T> command, T dt)
{ 
    Mat<T> r(state);    
    
    /*state v w */
    T v = state.get(4,1);
    T w = state.get(5,1);
    
    r.set( r.get(1,1) + v*cos(r.get(3,1))*dt, 1,1);
    r.set( r.get(2,1) + v*sin(r.get(3,1))*dt, 2,1);
    
    T angle = (r.get(3,1) + dt*w);
    
    while(angle > (T)PI)
    {
         angle -= 2.0f*(T)PI;                 
    }
    while(angle <= -(T)PI)
    {
        angle += 2.0f*(T)PI;                
    }
    
    r.set( angle, 3,1);
    
    #ifndef PROPAGATEVW?
    r.set( (T)0, 4,1);
    r.set( (T)0, 5,1);
    #endif
    
    return r;
}

template<typename T>
Mat<T> sensorModel( Mat<T> state, Mat<T> command, Mat<T> d_state, T dt)
{    
	Mat<T> r(2,1);
	r.set( state.get(4,1), 1,1);
	r.set( state.get(5,1), 2,1);
	
    return r;
}

template<typename T>
Mat<T> jmotionModel( Mat<T> state, Mat<T> command, T dt)
{    
    Mat<T> r((T)0,state.getLine(),state.getLine());
    #ifndef PROPAGATEVW?    
    for(int i=state.getLine();i--;)	r.set((T)1, i+1,i+1);
    #else
    for(int i=state.getLine()-2;i--;)	r.set((T)1, i+1,i+1);
    #endif
    
    /*state v w */
    T v = state.get(4,1);
    T w = state.get(5,1);
    T angle = state.get(3,1);
    
	r.set( -v*cos(angle)*w, 1,3);
	r.set( v*sin(angle)*w, 2,3);
	r.set( cos(angle), 1,4);
	r.set( sin(angle), 2,4);
	r.set( dt, 3,5);
    
    return r;
}


template<typename T>
Mat<T> jsensorModel( Mat<T> state, Mat<T> command, Mat<T> d_state, T dt)
{
    Mat<T> r((T)0,2,state.getLine());
    
	r.set( (T)1, 1,4);
	r.set( (T)1, 2,5);
    
    return r;
}
#else
//#define COMPASS_FUSION_USE	//TODO:ADD COMPASS FUSION TO NON HOLONOMIC SETTINGS... ADD SENSOR AND JSENSOR FONCTION.
#endif

void Euler2Quaternion(geometry_msgs::Quaternion* q, float roll, float pitch, float yaw)
{
	Quat qq = Euler2Qt(roll,pitch,yaw);
	q->x = qq.x;
	q->y = qq.y;
	q->z = qq.z;
	q->w = qq.w;
}



class WIDL
{

	protected :
	
	int rate;
	ros::NodeHandle nh;
	
	ros::Publisher deltaPose_pub;
	ros::Publisher globalPose_pub;
	ros::Publisher path_pub;
	ros::Publisher pose_stamped_pub;
	ros::Publisher EKFpath_pub;
	ros::Publisher EKFpose_stamped_pub;
	
	ros::Subscriber ups_scan_sub;
#ifdef COMPASS_FUSION_USE	
	ros::Subscriber	compass_sub;
#endif
	//DATA :
	geometry_msgs::Pose2D deltaPose;
	geometry_msgs::Pose2D globalPose;
	nav_msgs::Path path;
	nav_msgs::Path EKFpath;
	
	
#ifdef COMPASS_FUSION_USE
	float lastvalue;
	vector<geometry_msgs::Pose2D> compasses;
	bool newCompassReceived;
	int nbrCompassReceived;
	int compass_queue_size;
	
	Mat<float> RC;
	float varnoise_obs_thetaCOMPASS;
#endif	
	
	int scan_queue_size;	//default : 2.
	
	vector<sensor_msgs::LaserScan> ups_scans;
	
	//New scans that are being received are stored in those std::vector in wait of being use.
	
	int nbrScansReceived;
	bool newScansReceived;
	sensor_msgs::LaserScan ups_scansInUse[2];
	
	//handlers for the scans that are being used currently.
	int nbrPoints[2];
	vector<Mat<float> > pointsCartesianPose[2];
	vector<Mat<float> > pointsPolarPose[2];

	
	vector<float> incidenceAngles[2];
	vector<float> delta[2];
	
	Mat<float> distMat;
	Mat<float> statsSLAM;
	Mat<float> stats;
	Mat<int> matches[2];
	
	int n;	//nbr of range values...
	int nbrIt;
	
	//UNCERTAINTY :
	float sigma2Theta;
	float sigma2L;
	float beta;
	vector<Mat<float> > noiseP[2];
	vector<Mat<float> > corrP[2];
	//next one is to be computed once the matching is done so there is no propagation from one iteration to another...
	vector<Mat<float> > matchP[2];
	vector<Mat<float> > matchPinv[2];
	Mat<float> PDelta;
	vector<Mat<float> > qk;		//variables that are to be reused to compute the whole estimation-related stuff.
	Mat<float> Pp;				//Diplacement estimation Covariance Matrix
	Mat<float> PpTheta;
	float rT;					//Angular Deplacement Covariance Matrix
	Mat<float> Pest;			//Estimation Covariance matrix
	
	Mat<float> J;
	
	//------------------------------------------
	//-----------------------------------------
	//EEKF :
#ifndef CORRECTMOTIONMODEL	
	int nbrstate;
	int nbrcontrol;
	int nbrobs;
	float dt;
	float stdnoise;
	float stdnoise_obs;
	bool ext;
	bool filteron;
	bool noise;

	Mat<float> EKFPose;  
	Mat<float> EKFPoseCovar;  
	EEKF<float>* instanceEEKF;
	Mat<float> Q;
	Mat<float> R;
	float varnoise_obs_theta;
#else
	int nbrstate;
	int nbrcontrol;
	int nbrobs;
	float dt;
	float stdnoise;
	float stdnoise_obs;
	bool ext;
	bool filteron;
	bool noise;

	Mat<float> EKFPose;  
	Mat<float> EKFPoseCovar;  
	EEKF<float>* instanceEEKF;
	Mat<float> Q;
	Mat<float> R;
	float varnoise_obs_theta;
#endif	
	//------------------------------------------
	//-----------------------------------------
	
	
#ifdef OCCUPANCY_GRID_USE
	//------------------------------------------
	//------------------------------------------
	//	Occupancy GRID
	bool initGrid;
	Grid* instanceGrid;
	Grid* instanceGridInstantaneous;
	float precX;
	float precY;
	
#ifdef NAVMSGSGRID
	nav_msgs::OccupancyGrid msgs_grid;
	ros::Publisher	msgs_grid_pub;
#endif	
	//------------------------------------------
	//------------------------------------------			
#endif
	
	public :

#ifdef OCCUPANCY_GRID_USE	
	WIDL(int nbrIt_ = 1, int rate_ = 100, float precX_ = 0.05f, float precY_ = 0.05f, int scan_queue_size_ = 2 ) : rate(rate_), scan_queue_size(scan_queue_size_)
#else	
	WIDL(int nbrIt_ = 1, int rate_ = 100, int scan_queue_size_ = 2 ) : rate(rate_), scan_queue_size(scan_queue_size_)
#endif	
	{
		ups_scan_sub = nh.subscribe("/scan_extmedian_filter", 10, &WIDL::callback_ups, this);
	
#ifdef COMPASS_FUSION_USE	
		lastvalue = 0.0f;
		compass_sub = nh.subscribe("/COMPASSListener/compass", 10, &WIDL::callback_compass, this);
		nbrCompassReceived = 0;
		newCompassReceived = false;
		compass_queue_size = scan_queue_size_;
		varnoise_obs_thetaCOMPASS = 1e-20;	//very precise :)!!!
#endif
			
		deltaPose_pub = nh.advertise<geometry_msgs::Pose2D>("WIDL/DeltaPose", 10);
		globalPose_pub = nh.advertise<geometry_msgs::Pose2D>("WIDL/Pose", 10);
		path_pub = nh.advertise<nav_msgs::Path>("WIDL/Path",10);
		pose_stamped_pub = nh.advertise<geometry_msgs::PoseStamped>("WIDL/POSESTAMPED",10);
		EKFpath_pub = nh.advertise<nav_msgs::Path>("WIDL/EKFPath",10);
		EKFpose_stamped_pub = nh.advertise<geometry_msgs::PoseStamped>("WIDL/EKFPOSESTAMPED",10);
		
		nbrScansReceived = 0;
		newScansReceived = false;
		
		globalPose.x = 0.0f;
		globalPose.y = 0.0f;
		globalPose.theta = 0.0f;
		
		n=0;
		nbrIt = nbrIt_;
		
		//UNCERTAINTY :
		//sigma2Theta = pow(3.6e-2,2);	//(1e-4)^2 in paper
		sigma2Theta = pow(PI/50,2);	//(1e-4)^2 in paper
		sigma2L = pow( 0.001f, 2);	//(5mm)^2 in paper
		//sigma2L = pow( 6.0e-4, 2);	//(5mm)^2 in paper
		
		//these values are good for our current LRF.
		// especially the one in sigma2L which scrumbled the scale of the movement estimation.
		// test for sigma2Theta since this one is less accurate that expectations :
		
		beta = 0.36f*PI/180.f;	//yet we compute the delta's without using this value, so it is unnecessary.
		//sigma2Theta = pow(beta*100,2);	//odd value with that...
		//--------------------------------------------------------
		//--------------------------------------------------------
		//EKF :
#ifndef	CORRECTMOTIONMODEL				
		nbrstate = 6;
		nbrcontrol = 0;
		nbrobs = 3;
		dt = 1;
		
#ifndef 	doubleMatchOnlyEstimation		
		stdnoise = 2e-5;
		stdnoise_obs = 2e-4;
		float varnoise_obs_x = 4e-6;
		float varnoise_obs_y = 2e-6;
		varnoise_obs_theta = 3e-5;
#else
	#ifndef iterativeHAYAI
		stdnoise = 1e-4;
		stdnoise_obs = 1e-3;	//unused...
		float varnoise_obs_x = 6e-8;
		float varnoise_obs_y = 2e-7;
		varnoise_obs_theta = 3e-6;
	#else
		#ifndef BOX
		stdnoise = 1.5e-6;
		stdnoise_obs = 1e-4;	//unused...
		float varnoise_obs_x = 1.1e-5;
		float varnoise_obs_y = 3.7e-6;
		varnoise_obs_theta = 1.2e-7;
		float varnoise_thetaxy = 1e-4;
		#else
		stdnoise = 1.0e-7;
		stdnoise_obs = 1e-6;	//unused...
		float varnoise_obs_x = 8.2e-9;
		float varnoise_obs_y = 8.4e-8;
		varnoise_obs_theta = 4.6e-6;
		#endif
	#endif
#endif		
		ext = false;
		filteron = true;
		noise = false;

		EKFPose = Mat<float>((float)0,nbrstate,1);  
		EKFPoseCovar = Mat<float>((float)0,nbrstate,nbrstate);  
		instanceEEKF = new EEKF<float>(nbrstate,nbrcontrol,nbrobs,dt,stdnoise,EKFPose,ext,filteron,noise);

		Mat<float> A((float)0,nbrstate,nbrstate);
		for(int i=1;i<=nbrstate/2;i++)	A.set((float)1,i,i);
		//unstable if the velocity is propagated...
		for(int i=1;i<=nbrstate/2;i++)	A.set((float)dt,i,nbrstate/2+i);
		A.afficher();
		instanceEEKF->initA(A);
		
		Mat<float> C((float)0,nbrobs,nbrstate);
		for(int i=1;i<=nbrobs;i++)	C.set((float)1,i,nbrobs+i);
		C.afficher();
		instanceEEKF->initC(C);
		
		/*
		Mat<float> B((float)0,nbrstate,nbrcontrol);
		for(int i=1;i<=nbrcontrol;i++)	B.set((float)1,nbrcontrol+i,i);
		B.afficher();
		instanceEEKF->initB(B);
		*/
		
		Q = Mat<float>(0.0f,nbrstate,nbrstate);
		for(int i=1;i<=nbrstate;i++)	Q.set( stdnoise, i,i);
		
		/*
		Q.set( varnoise_thetaxy*1e-1, 3,1);
		Q.set( varnoise_thetaxy*1e-1, 3,2);
		Q.set( varnoise_thetaxy*1e-1, 1,3);
		Q.set( varnoise_thetaxy*1e-1, 2,3);
		*/
		
		Q.afficher();
		instanceEEKF->initQ(Q);
		
		R = Mat<float>(0.0f,nbrobs,nbrobs);
		R.set( varnoise_obs_x, 1,1);
		R.set( varnoise_obs_y, 2,2);
		R.set( varnoise_obs_theta, 3,3);
		
		/*
		R.set( varnoise_thetaxy, 3,1);
		R.set( varnoise_thetaxy, 3,2);
		R.set( varnoise_thetaxy, 1,3);
		R.set( varnoise_thetaxy, 2,3);
		*/
		
		R.afficher();
		instanceEEKF->initR(R);

#ifdef COMPASS_FUSION_USE
		//--------------
		//		EKF FUSION : COMPASS OBSERVATION MODEL
		//--------------
		//Model 1 :
		RC = Mat<float>(0.0f,nbrobs,nbrobs);
		RC.set( varnoise_obs_x, 1,1);
		RC.set( varnoise_obs_y, 2,2);
		RC.set( varnoise_obs_thetaCOMPASS, 3,3);
		RC.afficher();
		//instanceEEKF->initR(R);
#endif				

#else
		nbrstate = 5;
		nbrcontrol = 0;
		nbrobs = 2;
		//dt = 1;
		dt = 1.0f/rate;
		
#ifndef 	doubleMatchOnlyEstimation		
		stdnoise = 2e-5;
		stdnoise_obs = 2e-4;
		float varnoise_obs_x = 4e-6;
		float varnoise_obs_y = 2e-6;
		varnoise_obs_theta = 3e-5;
#else
	#ifndef iterativeHAYAI
		stdnoise = 1e-4;
		stdnoise_obs = 1e-3;	//unused...
		float varnoise_obs_x = 6e-8;
		float varnoise_obs_y = 2e-7;
		varnoise_obs_theta = 3e-6;
	#else
		#ifndef BOX
		stdnoise = 1.5e-6;
		stdnoise_obs = 1e-4;	//unused...
		float varnoise_obs_x = 1.1e-5;
		float varnoise_obs_y = 3.7e-6;
		varnoise_obs_theta = 1.2e-7;
		float varnoise_thetaxy = 1e-4;
		#else
		stdnoise = 1.0e-7;
		stdnoise_obs = 1e-6;	//unused...
		float varnoise_obs_x = 8.2e-9;
		float varnoise_obs_y = 8.4e-8;
		varnoise_obs_theta = 4.6e-6;
		#endif
	#endif
#endif		

		float varnoise_obs_v = varnoise_obs_x;
		float varnoise_obs_w = varnoise_obs_theta;
		stdnoise = 1e-7; 
		
		ext = true;
		filteron = true;
		noise = false;

		EKFPose = Mat<float>((float)0,nbrstate,1);  
		EKFPoseCovar = Mat<float>((float)0,nbrstate,nbrstate);  
		instanceEEKF = new EEKF<float>(nbrstate,nbrcontrol,nbrobs,dt,stdnoise,EKFPose,ext,filteron,noise);
		
		instanceEEKF->initMotion( motionModel);
		instanceEEKF->initSensor( sensorModel);
		instanceEEKF->initJMotionFunction( jmotionModel);
		instanceEEKF->initJSensorFunction( jsensorModel);
		
		Q = Mat<float>(0.0f,nbrstate,nbrstate);
		for(int i=1;i<=nbrstate;i++)	Q.set( stdnoise, i,i);
		
		
		Q.afficher();
		instanceEEKF->initQ(Q);
		
		R = Mat<float>(0.0f,nbrobs,nbrobs);
		R.set( varnoise_obs_v, 1,1);
		R.set( varnoise_obs_w, 2,2);
		
		R.afficher();
		instanceEEKF->initR(R);

//TODO : incorporate COMPASS FUSION :
/*
#ifdef COMPASS_FUSION_USE
		//--------------
		//		EKF FUSION : COMPASS OBSERVATION MODEL
		//--------------
		//Model 1 :
		RC = Mat<float>(0.0f,nbrobs,nbrobs);
		RC.set( varnoise_obs_x, 1,1);
		RC.set( varnoise_obs_y, 2,2);
		RC.set( varnoise_obs_thetaCOMPASS, 3,3);
		RC.afficher();
		//instanceEEKF->initR(R);
#endif
*/
#endif
		//--------------------------------------------------------
		//--------------------------------------------------------

#ifdef OCCUPANCY_GRID_USE
		//--------------------------------------------------------
		//--------------------------------------------------------
		//		Occupancy GRID
		
		initGrid = false;
		precX = precX_;
		precY = precY_;
		
		instanceGrid = new Grid(precX,precY, 0);
		instanceGridInstantaneous = new Grid(precX,precY, 1);
		
#ifdef NAVMSGSGRID
		msgs_grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("WIDL/OccupancyGrid", 10);
#endif	
#endif
		//--------------------------------------------------------
		//--------------------------------------------------------
		
		J = Mat<float>(0.0f, 2,2);
		J.set( -1.0f, 1,2);
		J.set( 1.0f, 2,1);
		
		//--------------------------------------------------------
		
		this->mainLoop();
	}
	
	~WIDL()
	{
		delete instanceEEKF;
		
#ifdef OCCUPANCY_GRID_USE
		delete instanceGrid;
		delete instanceGridInstantaneous;
#endif		
	}
	
	void callback_ups(sensor_msgs::LaserScan ups_scan)
	{
		ups_scans.insert( ups_scans.begin(), ups_scan);
		
		if(ups_scans.size() > scan_queue_size)
			ups_scans.pop_back();
			
		newScansReceived = true;
		nbrScansReceived++;
	}
	
#ifdef COMPASS_FUSION_USE	
	void callback_compass(geometry_msgs::Pose2D compassPose)
	{
		compasses.insert( compasses.begin(), compassPose);
		
		if(compasses.size() > compass_queue_size)
			compasses.pop_back();
			
		newCompassReceived = true;
		nbrCompassReceived++;
		
		//OBSERVATION IN EKF FILTER IS MADE HERE :
		//--------------------		
		Mat<float> updatePose(3,1);
		updatePose.set(0.0f,1,1);
		updatePose.set(0.0f,2,1);
		
		float currentvalue = compassPose.theta*PI/180.0f;
		while( currentvalue > PI)
		{
			currentvalue -= 2*PI;
		}
		while( currentvalue < -PI)
		{
			currentvalue += 2*PI;
		}
		
		float dtheta = currentvalue-lastvalue;
		
		//there is a problem when currentvalue is negative , e.g. -PI+epsilon radians, and lastvalue is positive, e.g. +PI-epsilon radians.
		if( (currentvalue < 0.0f && lastvalue > 0.0f) )
		{
			dtheta = -PI
		}
		else if(currentvalue > 0.0f && lastvalue < 0.0f)
		{
			
		}
		updatePose.set(dtheta, 3,1);
		lastvalue = compassPose.theta*PI/180.0f;
		// IN RADIANS !!!		
		
		instanceEEKF->measurement_Callback(updatePose);
		//instanceEEKF->setCommand(updatePose);
		instanceEEKF->state_Callback();
		EKFPose = instanceEEKF->getX();
		EKFPoseCovar = instanceEEKF->getSigma();
		
		ROS_INFO("EKF GLOBAL POSE WITH COMPASS FUSION : ");
		transpose(EKFPose).afficher();
		EKFPoseCovar.afficher();
		
		//---------------------------
	}
#endif

	
	void mainLoop()
	{
		int count_info = 100;
		
#ifdef debug_lvl6		
		//Gestion ecriture dans un fichier :
		string filepath("/home/kevidena/ROS/sandbox/WIDL/src/log.txt");
		FILE* log = fopen(filepath.c_str(), "w+");
		if(log == NULL)
		{
			cout << "ERROR : cannot open the file LOG." << endl;
			exit(1);
		}
		else
			cout << "File opened LOG." << endl;
#endif
	
		while(nbrScansReceived < 1)
		{
			if(count_info>100)
			{
				ROS_INFO("WIDL::Initialization : ...");
				count_info = 0;
			}
			count_info++;
			
			ros::spinOnce();
		}
		
		//initialize the first pair of scans that are to be used :
		ups_scansInUse[0] = ups_scans[0];
		
		//initialize the first features extracted from the first pair of scans :
		nbrPoints[0] = preparePoints(ups_scansInUse[0], pointsCartesianPose[0], pointsPolarPose[0],delta[0],incidenceAngles[0]);
		prepareUncertainty(noiseP[0], corrP[0], pointsCartesianPose[0],pointsPolarPose[0], delta[0],incidenceAngles[0]);
		
		ROS_INFO("WIDL::Initialization : DONE.");
		
		while(nbrScansReceived < 2)
		{
			if(count_info>100)
			{
				ROS_INFO("WIDL::Waiting for scans in order to begin the loop ...");
				count_info = 0;
			}
			count_info++;
			
			ros::spinOnce();
		}
		
#ifdef debug_lvl2	
		//Gestion ecriture dans un fichier :
		string filepath("/home/kevidena/ROS/sandbox/WIDL/src/log.txt");
		FILE* log = fopen(filepath.c_str(), "w+");
		if(log == NULL)
		{
			cout << "ERROR : cannot open the file LOG." << endl;
			exit(1);
		}
		else
			cout << "File opened LOG." << endl;
			
		//------------------------------------------
		//------------------------------------------
			
		//Ecriture  :
    	n = (int)((ups_scans[0].angle_max - ups_scans[0].angle_min)/ups_scans[0].angle_increment);
    	
		for(int i=0;i<n;i++)	
		{
			stringstream s;
			s << ups_scans[0].ranges[i];
			s << endl;
			//cout << s.str();
			fputs( s.str().c_str(), log);
		}
	
		
		//------------------------------------
		//Fermeture du fichier :
		if(fclose(log) == EOF)
		{
			cout << "ERROR : cannot close the file." << endl;
			exit(1);
		}
		else
			cout << "File closed." << endl;
			
		//--------------------------------
		//--------------------------------
#endif		
		
		ros::Rate r(rate);	//100Hz	(idea : --> the same as the LRF in order to reduice noise propagation from this estimator).
		while(ros::ok())
		{
			
			statsSLAM = Mat<float>(0.0f,2,3);
			
			if(newScansReceived)
			{
				clock_t timer = clock();
				//--------------------------------
				//		settings :
				//--------------------------------
				newScansReceived = false;
				
				//--------------------------------
				//--------------------------------
				
				
				//--------------------------------
				//		slamProcess : initialization
				//--------------------------------
				clock_t timer_init = clock();
				
				ups_scansInUse[1] = ups_scansInUse[0];
				
				nbrPoints[1] = nbrPoints[0];
				pointsCartesianPose[1] = pointsCartesianPose[0];
				pointsPolarPose[1] = pointsPolarPose[0];
				
				incidenceAngles[1] = incidenceAngles[0];
				delta[1] = delta[0];
				noiseP[1] = noiseP[0];
				corrP[1] = corrP[0];
				
				//---------------------------------------

#ifdef debug_lvl1
				ROS_INFO("WIDL::EXECUTION INIT : %f Hz.", (float)(1.0/((float)(clock()-timer_init)/CLOCKS_PER_SEC)) );
				timer_init = clock();
#endif
				//initialize the first pair of scans that are to be used :
				ups_scansInUse[0] = ups_scans[0];
				//initialize the first features extracted from the first pair of scans :
				nbrPoints[0] = preparePoints(ups_scansInUse[0], pointsCartesianPose[0], pointsPolarPose[0],delta[0], incidenceAngles[0]);
				
#ifdef debug_lvl1				
				ROS_INFO("WIDL::EXECUTION INITPOINTS : PREPARETION : %f Hz.", (float)(1.0/((float)(clock()-timer_init)/CLOCKS_PER_SEC)) );
				timer_init = clock();
#endif				

				prepareUncertainty(noiseP[0], corrP[0], pointsCartesianPose[0],pointsPolarPose[0], delta[0],incidenceAngles[0]);
				
#ifdef debug_lvl1				
				ROS_INFO("WIDL::EXECUTION INITPOINTS : UNCERTAINTY : %f Hz.", (float)(1.0/((float)(clock()-timer_init)/CLOCKS_PER_SEC)) );
#endif				
				//------------------------------------------------------------------------------------------------
				//------------------------------------------------------------------------------------------------
				
				if(nbrPoints[0] != 0 && nbrPoints[1] != 0)
				{
#ifdef debug_lvl1				
					ROS_INFO("WIDL::NBR FEATURES : %d * %d = %d.", nbrPoints[0],nbrPoints[1], nbrPoints[0]*nbrPoints[1]);
#endif					
					//--------------------------------
					//		slamProcess : computation
					//-------------------------------
#ifdef debug_lvl1					
					clock_t timer_mat = clock();
#endif				

					//--------------------------------
					//		slamProcess : computation : distance Matrix
					//-------------------------------
					computeDistanceMatrix();
					
					
#ifdef debug_lvl1					
					ROS_INFO("WIDL::EXECUTION DITMAT : %f Hz.", (float)(1.0/((float)(clock()-timer_mat)/CLOCKS_PER_SEC)) );
					clock_t timer_matching = clock();
#endif				

					//--------------------------------
					//		slamProcess : computation : points matching vectors
					//-------------------------------
					pointsMatching();
					
					//initial guess :
					//float initTheta = angleComputationHistogram();
#ifndef optimHAYAIinit					
					float initTheta = 0.0f;
					ROS_INFO("WIDL::Initial Guess : Theta = %f.", initTheta);
#else					
					float initTheta = angleComputationHAYAI();
					ROS_INFO("WIDL::HAYAI Initial Guess : Theta = %f.", initTheta);
#endif					
					
					
					computeMatchingCovariance( rot2D(initTheta) );
					
#ifdef debug_lvl1					
					ROS_INFO("WIDL::EXECUTION MATCHING : %f Hz.", (float)(1.0/((float)(clock()-timer_matching)/CLOCKS_PER_SEC)) );
#endif
					
					//--------------------------------
					//		slamProcess : computation
					//-------------------------------
					estimateDeltaPose(initTheta);
					
					
#ifdef debug_lvl1
					ROS_INFO("WIDL::slamProcess : distances stats : mean = %f ; var = %f.", stats.get(1,1), stats.get(2,1) );
					//distMat.afficher();
					//matches[0].afficher();
					//matches[1].afficher();
#endif										
					
				}
				else
				{
					//TODO :
					deltaPose.x = 0.0f;
					deltaPose.y = 0.0f;
					deltaPose.theta = 0.0f;

#ifdef debug_lvl1					
					ROS_INFO("WIDL::slamProcess : not enough points ...");
#endif
				
				}
				
				//--------------------------------
				//--------------------------------
				
				
				//---------------------------------
				//		slamProcess : updating
				//---------------------------------
				Mat<float> updatePose(3,1);
				updatePose.set( cos(globalPose.theta)*deltaPose.x - sin(globalPose.theta)*deltaPose.y, 1,1);
				updatePose.set( sin(globalPose.theta)*deltaPose.x + cos(globalPose.theta)*deltaPose.y, 2,1);
				updatePose.set(  deltaPose.theta, 3,1);
				
				globalPose.x += updatePose.get(1,1);
				globalPose.y += updatePose.get(2,1);
				globalPose.theta += updatePose.get(3,1);
				
				//---------------------------
				// EEKF :
#ifndef CORRECTMOTIONMODEL				
				// Variances reorientation within the sensor frame :
				/*
				Mat<float> rot(rot2D(EKFPose.get(3,1)));
				rot = rot%rot;
				Mat<float> vQ(2,1);
				Mat<float> vR(2,1);
				vQ.set( Q.get(1,1), 1,1);
				vQ.set( Q.get(2,2), 2,1);
				vR.set( R.get(1,1), 1,1);
				vR.set( R.get(2,2), 2,1);
				vQ = rot*vQ;
				vR = rot*vR;
				
				Mat<float> tQ(Q);
				tQ.set( vQ.get(1,1), 1,1);
				tQ.set( vQ.get(2,1), 2,2);
				tQ.set( vQ.get(1,1), 4,4);
				tQ.set( vQ.get(2,1), 5,5);
				
				float ct = cos(EKFPose.get(1,1));
				float st = sin(EKFPose.get(2,1));
				tQ.set( vQ.get(1,1)-vQ.get(2,2)+ct*st*(Q.get(2,2)-Q.get(1,1)), 1,2);
				tQ.set( vQ.get(1,1)-vQ.get(2,2)+ct*st*(Q.get(2,2)-Q.get(1,1)), 2,1);
				tQ.set( vQ.get(1,1)-vQ.get(2,2)+ct*st*(Q.get(5,5)-Q.get(4,4)), 4,5);
				tQ.set( vQ.get(1,1)-vQ.get(2,2)+ct*st*(Q.get(5,5)-Q.get(4,4)), 5,4);
				
				Mat<float> tR(R);
				tR.set( vR.get(1,1), 1,1);
				tR.set( vR.get(2,1), 2,2);
				tR.set( vR.get(1,1)-vR.get(2,2)+ct*st*(R.get(2,2)-R.get(1,1)), 1,2);
				tR.set( vR.get(1,1)-vR.get(2,2)+ct*st*(R.get(2,2)-R.get(1,1)), 2,1);
				
				instanceEEKF->initQ(tQ);
				instanceEEKF->initR(tR);
				*/
				//--------------------------
				/*
				ROS_INFO("COVARIANCE MATRIX OF THE ESTIMATION : ");
				PDelta.afficher();
				Mat<float> tR(PDelta, 0.0f, 1,1, 3,3);
				tR.set(R.get(3,3), 3,3);
				instanceEEKF->initR(tR);
				*/
				//--------------------
				/*
				Mat<float> rot(rot2D(EKFPose.get(3,1)));
				Mat<float> vQ1(0.0f,2,2);
				Mat<float> vQ2(0.0f,2,2);
				Mat<float> vR(0.0f,2,2);
				vQ1.set( Q.get(1,1), 1,1);
				vQ1.set( Q.get(2,2), 2,2);
				vQ2.set( Q.get(4,4), 1,1);
				vQ2.set( Q.get(5,5), 2,2);
				vR.set( R.get(1,1), 1,1);
				vR.set( R.get(2,2), 2,2);
				vQ1 = rot*vQ1*transpose(rot);
				vQ2 = rot*vQ2*transpose(rot);
				vR = rot*vR*transpose(rot);
				
				Mat<float> tQ(0.0f,6,6);
				Mat<float> tR(0.0f,3,3);
				
				for(int i=2;i--;)
				{
					for(int j=2;j--;)
					{
						tQ.set( vQ1.get(i+1,j+1), i+1,j+1);
						tQ.set( vQ2.get(i+1,j+1), 3+i+1, 3+j+1);
						tR.set( vR.get(i+1,j+1), i+1,j+1);
					}
				}
				
				tQ.set( Q.get(3,3), 3,3);
				tQ.set( Q.get(6,6), 6,6);
				tR.set( R.get(3,3), 3,3);
				
				tQ.afficher();
				tR.afficher();
				instanceEEKF->initQ(tQ);
				instanceEEKF->initR(tR);
				*/
				//--------------------
#ifdef COMPASS_FUSION_USE
				updatePose.set( EKFPose.get(6,1), 3,1);
				//let's assume that the measure angular velocity is the registered one so that this observation as no undesired effect...
#endif											
				updatePose.set( cos(EKFPose.get(3,1))*deltaPose.x - sin(EKFPose.get(3,1))*deltaPose.y, 1,1);
				updatePose.set( sin(EKFPose.get(3,1))*deltaPose.x + cos(EKFPose.get(3,1))*deltaPose.y, 2,1);
				updatePose.set(  deltaPose.theta, 3,1);	
				instanceEEKF->measurement_Callback(updatePose);
				//instanceEEKF->setCommand(updatePose);
    			instanceEEKF->state_Callback();
    			EKFPose = instanceEEKF->getX();
    			EKFPoseCovar = instanceEEKF->getSigma();
#else
				//TODO : incorporation COMPASS FUSION...
				/*
				#ifdef COMPASS_FUSION_USE
				updatePose.set( EKFPose.get(6,1), 3,1);
				//let's assume that the measure angular velocity is the registered one so that this observation as no undesired effect...
				#endif											
				*/
				
				Mat<float> measurements(2,1);
				float xoffsetRLRF = 0.225f;	//offset in the x axis (robot frame) between O_robot and O_LRF : 22.5cm
				// v and w :
				//measurements.set( deltaPose.x*cos(EKFPose.get(3,1)) / dt, 1,1);
				//measurements.set( (1.0f/xoffsetRLRF)*(deltaPose.y - tan( EKFPose.get(3,1) ) * deltaPose.x)/dt, 2,1);
				
				//measurements.set( deltaPose.x / dt, 1,1);
				measurements.set( deltaPose.x*cos(deltaPose.theta) / dt, 1,1);
				measurements.set( deltaPose.theta/dt, 2,1);
				
				ROS_INFO("MEASURE : v = %f ; w = %f.", measurements.get(1,1), measurements.get(2,1) );
				
				instanceEEKF->measurement_Callback(measurements);
				//instanceEEKF->setCommand(updatePose);
    			instanceEEKF->state_Callback();
    			EKFPose = instanceEEKF->getX();
    			EKFPoseCovar = instanceEEKF->getSigma();
#endif
    			
    			ROS_INFO("EKF GLOBAL POSE : ");
    			transpose(EKFPose).afficher();
    			EKFPoseCovar.afficher();
    			
				//---------------------------
				
				while(globalPose.theta > PI)
				{
					globalPose.theta -= 2*PI;
				}
				while(globalPose.theta < -PI)
				{
					globalPose.theta += 2*PI;
				}
				
				ROS_INFO("WIDL::GLOBAL POSE : x = %f ; y = %f ; theta = %f", globalPose.x, globalPose.y, globalPose.theta);
				
				std_msgs::Header head_path;
				head_path.seq = nbrScansReceived/4;
				head_path.frame_id = "map";
				
				geometry_msgs::Pose path_pose;
				path_pose.position.x = globalPose.x;
				path_pose.position.y = globalPose.y;
				path_pose.position.z = (float)0;
				
				/*
				path_pose.orientation.x = (float)0;
				path_pose.orientation.y = (float)0;
				path_pose.orientation.z = globalPose.theta;
				path_pose.orientation.w = (float)1;
				*/
				Euler2Quaternion(&(path_pose.orientation), 0.0f,0.0f,globalPose.theta);
				
				geometry_msgs::PoseStamped path_poseStamped;
				path_poseStamped.header.stamp = ros::Time(0);
				path_poseStamped.header.frame_id = "map";
				path_poseStamped.pose = path_pose;
				
				path.poses.push_back(path_poseStamped);
				path.header.stamp = ros::Time(0);
				path.header.frame_id = "map";
				
				//--------------------------------
				//--------------------------------
				
				deltaPose_pub.publish(deltaPose);
				globalPose_pub.publish(globalPose);
				path_pub.publish(path);
				pose_stamped_pub.publish(path_poseStamped);
				
				path_pose.position.x = EKFPose.get(1,1);
				path_pose.position.y = EKFPose.get(2,1);
				path_pose.position.z = (float)0;
				
				/*
				path_pose.orientation.x = (float)0;
				path_pose.orientation.y = (float)0;
				path_pose.orientation.z = EKFPose.get(3,1);
				path_pose.orientation.w = (float)1;
				*/
				Euler2Quaternion(&(path_pose.orientation), 0.0f,0.0f,EKFPose.get(3,1));
				
				path_poseStamped.header.stamp = ros::Time(0);
				path_poseStamped.header.frame_id = "map";
				path_poseStamped.pose = path_pose;
				
				EKFpath.poses.push_back(path_poseStamped);
				EKFpath.header.stamp = ros::Time(0);
				EKFpath.header.frame_id = "map";
				
				EKFpath_pub.publish(EKFpath);
				EKFpose_stamped_pub.publish(path_poseStamped);
				
#ifdef 	OCCUPANCY_GRID_USE
				//---------------------------------
				//		slamProcess : MAPPING
				//---------------------------------
				clock_t timeGrid = clock();
				if(!initGrid)
				{
					initGrid = true;
					instanceGrid->init( /*state*/EKFPose, /*CovarianceMatrix*/EKFPoseCovar, /*observations*/pointsCartesianPose[0],pointsPolarPose[0]);
					instanceGridInstantaneous->init( /*state*/EKFPose, /*CovarianceMatrix*/EKFPoseCovar, /*observations*/pointsCartesianPose[0],pointsPolarPose[0]);
				}
				instanceGridInstantaneous->update( /*state*/EKFPose, /*CovarianceMatrix*/EKFPoseCovar, /*observations*/pointsCartesianPose[0],pointsPolarPose[0]);
				
				
#ifdef NAVMSGSGRID
				msgs_grid.header = head_path;
				
				nav_msgs::MapMetaData metagrid;
				metagrid.map_load_time = ros::Time(0);
				metagrid.resolution = precX;
#define instantaneousGRID				
#ifdef	instantaneousGRID				
				metagrid.width = instanceGridInstantaneous->getWidth();
				metagrid.height = instanceGridInstantaneous->getHeight();				
				Mat<int> origin(instanceGridInstantaneous->getOrigin());
				//origin of data(0,0)
				metagrid.origin.position.x = instanceGridInstantaneous->getXb(0,0);
				metagrid.origin.position.y = instanceGridInstantaneous->getYb(0,0);	
#else
				metagrid.width = instanceGrid->getWidth();
				metagrid.height = instanceGrid->getHeight();				
				Mat<int> origin(instanceGrid->getOrigin());		
				//origin of data(0,0)
				metagrid.origin.position.x = instanceGrid->getXb(0,0);
				metagrid.origin.position.y = instanceGrid->getYb(0,0);		
#endif				
				
				metagrid.origin.position.z = 0;				
				metagrid.origin.orientation = geometry_msgs::Quaternion();
				/*
				metagrid.origin.orientation.x = 0;
				metagrid.origin.orientation.y = 0;
				metagrid.origin.orientation.z = 0;
				metagrid.origin.orientation.w = 1;
				*/
				msgs_grid.info = metagrid;
				
				//data :
				msgs_grid.data.clear();
				//for(int j=0;j<metagrid.height;j++)
				for(int j=metagrid.height;j--;)
				{
					
					//for(int i=0;i< metagrid.width;i++)
					for(int i=metagrid.width;i--;)
					{
						//msgs_grid.data[i*metagrid.height+j] = instanceGrid->getOccP(i,j)*100;
#ifdef	instantaneousGRID										
						msgs_grid.data.insert(msgs_grid.data.begin(), instanceGridInstantaneous->getOccP(i,j)*100 );
#else
						msgs_grid.data.insert(msgs_grid.data.begin(), instanceGrid->getOccP(i,j)*100 );
#endif						
					}
				}
				
				msgs_grid_pub.publish( msgs_grid);
#endif
				ROS_INFO("WIDL::Occupancy GRID : %f Hz.", (float)(CLOCKS_PER_SEC/((float)clock()-timeGrid)) );
#endif				
				//-------------------------------
				//		STATS
				//-------------------------------
				
				statsSLAM.set(statsSLAM.get(1,1)+ 0, 1,1);//deltaPose.y;
				statsSLAM.set(statsSLAM.get(2,1) + pow( deltaPose.x, 2), 2,1);
				
				statsSLAM.set(statsSLAM.get(1,2)+ 0, 1,2);//deltaPose.y;
				statsSLAM.set(statsSLAM.get(2,2) + pow( deltaPose.y, 2), 2,2);
				
				statsSLAM.set(statsSLAM.get(1,3)+ 0, 1,3);//deltaPose.y;
				statsSLAM.set(statsSLAM.get(2,3) + pow( deltaPose.theta, 2), 2,3);
				
				ROS_INFO("WIDL::STATS SLAM : nbrIt : %d ; STATS :", nbrScansReceived);
				((1.0f/(nbrScansReceived - 1))*statsSLAM).afficher();
				
				//-------------------------------------
				//-------------------------------------
					
				ROS_INFO("WIDL::EXECUTION : %f Hz.", (float)(1.0/((float)(clock()-timer)/CLOCKS_PER_SEC)) );	
				
#ifdef debug_lvl6	
				//Ecriture  :
				stringstream s;			
				for(int i=1;i<=3;i++)	
				{
					s << updatePose.get(i,1) << " | " ;
				}
				s << endl;
				//cout << s.str();
				fputs( s.str().c_str(), log);
#endif		

				
		#ifdef debug_lvl3	
				//Gestion ecriture dans un fichier :
				string filepath0("/home/kevidena/ROS/sandbox/WIDL/src/log0.txt");
				FILE* log0 = fopen(filepath0.c_str(), "w+");
				if(log0 == NULL)
				{
					cout << "ERROR : cannot open the file LOG0." << endl;
					exit(1);
				}
				else
					cout << "File opened LOG0." << endl;
			
				string filepath1("/home/kevidena/ROS/sandbox/WIDL/src/log1.txt");
				FILE* log1 = fopen(filepath1.c_str(), "w+");
				if(log0 == NULL)
				{
					cout << "ERROR : cannot open the file LOG1." << endl;
					exit(1);
				}
				else
					cout << "File opened LOG1." << endl;
				//------------------------------------------
				//------------------------------------------
			
				//Ecriture  :
				for(int i=0;i<nbrPoints[0];i++)	
				{
					stringstream s;
					s << pointsCartesianPose[0][i].get(1,1) << " , " << pointsCartesianPose[0][i].get(2,1) << " , " << matches[0].get(1,i+1);
					s << " | " ;
					s << matchP[0][i].get(1,1) << " , " << matchP[0][i].get(2,2) ;
					s << " | " ;
					s << matchPinv[0][i].get(1,1) << " , " << matchPinv[0][i].get(2,2) ;
					s << endl;
					//cout << s.str();
					fputs( s.str().c_str(), log0);
				}
		
				for(int i=0;i<nbrPoints[1];i++)	
				{
					stringstream s;
					s << pointsCartesianPose[1][i].get(1,1) << " , " << pointsCartesianPose[1][i].get(2,1) << " , " << matches[1].get(1,i+1);
					s << " | " ;
					s << matchP[1][i].get(1,1) << " , " << matchP[1][i].get(2,2) ;
					s << " | " ;
					s << matchPinv[1][i].get(1,1) << " , " << matchPinv[1][i].get(2,2) ;
					s << endl;
					//cout << s.str();
					fputs( s.str().c_str(), log1);
				}
	
		
				//------------------------------------
				//Fermeture du fichier :
				if(fclose(log0) == EOF)
				{
					cout << "ERROR : cannot close the file LOG0." << endl;
					exit(1);
				}
				else
					cout << "File closed." << endl;
			
				if(fclose(log1) == EOF)
				{
					cout << "ERROR : cannot close the file LOG1." << endl;
					exit(1);
				}
				else
					cout << "File closed." << endl;
				//--------------------------------
				//--------------------------------
		#endif		

				
			}
			
			
			
			
			r.sleep();
			ros::spinOnce();
		
		}
		
		
#ifdef debug_lvl6		
		//------------------------------------
		//Fermeture du fichier :
		if(fclose(log) == EOF)
		{
			cout << "ERROR : cannot close the file." << endl;
			exit(1);
		}
		else
			cout << "File closed." << endl;
	
		//--------------------------------
		//--------------------------------
#endif
		
				
	}
	
	int preparePoints(const sensor_msgs::LaserScan& ups_scan, vector<Mat<float> >& pointsCartesianPose_, vector<Mat<float> >& pointsPolarPose_, vector<float>& delta_, vector<float>& incidenceAngles_)
	{
		pointsCartesianPose_.clear();
		pointsPolarPose_.clear();
		delta_.clear();
		incidenceAngles_.clear();
		
		int nbrpoint = 0;
		
		n = (int)((ups_scan.angle_max-ups_scan.angle_min)/ups_scan.angle_increment);
		float a_inc = ups_scan.angle_increment;
		Mat<float> tempCart(2,1);
		Mat<float> tempPol(2,1);
		
		float roffset = 3.8f;
		float med_size = ups_scan.intensities[1];
		/*
		float ect = 0.0f;
		for(int i=n;i--;)	ect += pow(ups_scan.ranges[i]-offset, 2);
		ect /= n-1;
		ect = sqrt(ect);
		*/
		float deltaMean = 0.0f;
		
		for(int i=n;i--;)
		{
			if(i>med_size+1 || i<n-med_size-1)
			{
			
				if(ups_scan.ranges[i] <= roffset)
				{
					//we know that this point is not due to noise..
					nbrpoint++;
			
					tempPol.set(ups_scan.ranges[i], 1,1);
					tempPol.set((ups_scan.angle_min + i*a_inc), 2,1);
					
					tempCart.set( tempPol.get(1,1)*cos(tempPol.get(2,1)), 1,1);
					tempCart.set( tempPol.get(1,1)*sin(tempPol.get(2,1)), 2,1);
					
					pointsPolarPose_.insert(pointsPolarPose_.begin(), tempPol);
					pointsCartesianPose_.insert(pointsCartesianPose_.begin(), tempCart);
			
					if(pointsPolarPose_.size() >1)
					{
						float tempDelta = norme2(pointsCartesianPose_[0]-pointsCartesianPose_[1]);
						delta_.insert(delta_.begin(), tempDelta );
						deltaMean += tempDelta;
					}
					//regularization at the end with a mean value.
				}
			}
			
			if(nbrpoint >= maxNBRPOINT)
			{
				ROS_INFO("WIDL::POINT CAPACITY HAS REACHED THE LIMIT : %d.", maxNBRPOINT);
				n=maxNBRPOINT;
				i=0;
			}
		
		}
		delta_.insert(delta_.begin(), deltaMean/nbrpoint);
		
		//---------------------------------------
		//---------------------------------------
		//		INCIDENCE ANGLES : computation
		//---------------------------------------
		//little hack in order to minimize the computation cost :
		//the incidence angle is computed with a computed line being 
		//a mean over the scan points that yields within 10% of the surrounding scan points.
		//USING RANSAC maybe:
		//USING HOUGH TRANSFORM :
		float Rmax = 4.0f;
		float Rprecision = 0.1f;
		float Aprecision = PI/50;	//1%
		
#ifdef debug_lvl1		
		clock_t timerHG = clock();
#endif		
		HoughT instanceHT( pointsCartesianPose_, Rmax, Rprecision,Aprecision);
		vector<Mat<float> >  bestMatch = instanceHT.getLines();
#ifdef debug_lvl1					
		ROS_INFO("WIDL::EXECUTION Line Extraction HOUGH TRANSFORM : %f Hz.", (float)(1.0/((float)(clock()-timerHG)/CLOCKS_PER_SEC)) );
#endif			
		
		//watch out : we no longer use the scans entirely but only the scanned points with correct values...
		for(int k=nbrpoint;k--;)
		{
			//let's associate the most probable incidence angle to each point :
			//float a_line = instanceHT.getLine4Idx(k).get(2,1);
			float a_line = bestMatch[k].get(2,1);
			float a_pointRad = pointsPolarPose_[k].get(2,1);
			//already in radian...
			//*PI/180.0f;
			//bounded to -PI/2 and PI/2.
			
			float t_incA;
			
			if(a_pointRad > 0.0f)
			{
				if(a_line <= PI/2)
					t_incA = fabs_( (a_line+PI/2) - a_pointRad);
				else
					t_incA = fabs_(3*PI/2+a_pointRad-a_line);
			}
			else
			{
				a_line = PI - a_line;
				
				if(a_line <= PI/2)
					t_incA = fabs_( (a_line+PI/2) - a_pointRad);
				else
					t_incA = fabs_(3*PI/2+a_pointRad-a_line);
			}
			
			incidenceAngles_.insert( incidenceAngles_.begin(), t_incA );
		}
		
		//---------------------------------------
		//---------------------------------------
		
		return nbrpoint;
	
	
	}
	
	void prepareUncertainty(vector<Mat<float> >& noiseP_, vector<Mat<float> >& corrP_, const vector<Mat<float> >& pointsCartesianPose_, const vector<Mat<float> >& pointsPolarPose_, const vector<float>& delta_, const vector<float>& incidenceAngles_)
	{
		noiseP_.clear();
		corrP_.clear();
		
		int nbrpoints = pointsCartesianPose_.size();

		
		for(int k=nbrpoints;k--;)
		{
			//let us compute the correspondant covariances matrixes for each point :
			//------------------------------------------
			float theta = pointsPolarPose_[k].get(2,1);
			//Correspondence noise covariance :			
			float etaA = theta + incidenceAngles_[k];
			Mat<float> tC_P(2,2);
			float cC = cos(etaA);
			float sC = sin(etaA);
			tC_P.set( pow( cC, 2), 1,1);
			tC_P.set( cC*sC, 1,2);
			tC_P.set( cC*sC, 2,1);
			tC_P.set( pow( sC, 2), 2,2);
			
			//---------------------
			//		TEST
			//------------------
			etaA = -theta-incidenceAngles_[k]+PI;
			cC = cos(etaA);
			sC = sin(etaA);
			tC_P.set( pow( cC, 2), 1,1);
			tC_P.set( cC*sC, 1,2);
			tC_P.set( cC*sC, 2,1);
			tC_P.set( pow( sC, 2), 2,2);
			
			//-----------------
			
			float dp = delta_[k+1];
			float dm = delta_[k];
			Mat<float> corrPTemp((float)((pow(dp,3)+pow(dm,3))/(3*(dp+dm))) * tC_P);
			//ROS_INFO("CORR P  :  %d. ", k);
			//corrPTemp.afficher();
			
			
			corrP_.insert( corrP_.begin(), corrPTemp );
			
			//------------------------------------------
			//------------------------------------------
			//Sensor noise covariance :
			float cN = cos(theta);
			float sN = sin(theta);
			Mat<float> tN_P(2,2);
			float range = pointsPolarPose_[k].get(1,1);
			float p1 = pow(range,2)*sigma2Theta/2;
			float p2 = sigma2L/2;
			//float p3 = 
			tN_P.set( p1*2*pow(sN,2)+p2*2*pow(cN,2), 1,1);
			tN_P.set( p1*(-sin(2*theta))+p2*sin(2*theta), 1,2);
			tN_P.set( p1*2*pow(cN,2)+p2*2*pow(sN,2), 2,2);
			tN_P.set( tN_P.get(1,2), 2,1);
			
			noiseP_.insert( noiseP_.begin(), tN_P);
			
			//-----------------------------------------
			//-----------------------------------------
			
			
		}
	
	}
	
	float distPruningIDP(const Mat<float>& p0, const Mat<float>& p1, float PolarTresh, float RangeTresh)
	{
		bool withinPolarRange = (abs(p0.get(2,1)-p1.get(2,1)) < PolarTresh ? true : false);
		bool withinRangeRange = (abs(p0.get(1,1)-p1.get(1,1)) < RangeTresh ? true : false);
		return (/*withinPolarRange &&*/ withinRangeRange ? 0.0f : INF);
		//too much pruning sometimes and thus no possible match...
		//TODO : test for it in computeDistanceMatrix... and add a "unmatched" point
	}
	
	
	void computeDistanceMatrix()
	{
		distMat = Mat<float>(nbrPoints[0],nbrPoints[1]);
		
		float w1 = 1.0f;
		float w2 = 1.0f;
		float w3 = 1.0f;
		float w4 = 1.0f;
		
		//TODO TRESH :
		float PolarTresh = PI/2;
		float RangeTresh = 0.2f;
		
		
		for(int i=nbrPoints[0];i--;)
		{
			for(int j=nbrPoints[1];j--;)
			{
				float dist = sqrt( w1*pow(pointsPolarPose[0][i].get(1,1)-pointsPolarPose[1][j].get(1,1),2) + w2*pow(pointsPolarPose[0][i].get(2,1)-pointsPolarPose[1][j].get(2,1), 2) );
				dist += sqrt( w3*pow(pointsCartesianPose[0][i].get(1,1)-pointsCartesianPose[1][j].get(1,1),2) + w3*pow(pointsCartesianPose[0][i].get(2,1)-pointsCartesianPose[1][j].get(2,1), 2) );
				dist += distPruningIDP(pointsPolarPose[0][i], pointsPolarPose[1][j], PolarTresh,RangeTresh);
				distMat.set( dist , i+1,j+1);
				
			}
		}
		
	}
	
	void pointsMatching()
	{
		matches[0] = Mat<int>(1,nbrPoints[0]);
		matches[1] = Mat<int>(1,nbrPoints[1]);
		
		int range_heur = RANGE_HEUR;
		int idx1 = 0;
		int idx2 = 0;
		
		//TODO : estimate this value :
//#define 	TRESHMATCH
#ifdef TRESHMATCH		
		float wdistTreshold = 1.0f;	//10 cm;
#endif		

		for(int k=nbrPoints[0];k--;)
		{
			if(k+1-range_heur <= 1)
				idx1 = 1;
			else
				idx1 = k+1-range_heur;
			
			if(k+1+range_heur >= nbrPoints[1])
				idx2 = nbrPoints[1];
			else
				idx2 = k+1+range_heur;
			
			Mat<float> indexvalmin( idmin( extract(&distMat, k+1, idx1, k+1, idx2) ) );
#ifdef TRESHMATCH			
			if(indexvalmin.get(1,1) > wdistTreshold)
			{
				ROS_INFO("element : %d, in current scan : unmatched : dist : %f", k, indexvalmin.get(1,1));
				indexvalmin.set( nbrPoints[0], 2,1);
			}
#endif			
			matches[0].set( (idx1-1) + indexvalmin.get(2,1), 1,k+1);						
			
			
			//idx of the column
		}
		
		for(int k=nbrPoints[1];k--;)
		{
			if(k+1-range_heur <=1)
				idx1 = 1;
			else
				idx1 = k+1-range_heur;
			
			if(k+1+range_heur >=nbrPoints[0])
				idx2 = nbrPoints[0];
			else
				idx2 = k+1+range_heur;
				
			matches[1].set( (idx1-1) + idmin( extract(&distMat, idx1, k+1, idx2, k+1) ).get(1,1), 1,k+1);	
#ifdef debug_lvl4			
			if(k==nbrPoints[1]/2)
			{
				cout << idx1-1 << endl;
				extract(&distMat, idx1, k+1, idx2, k+1).afficher();
				idmin( extract(&distMat, idx1, k+1, idx2, k+1) ).afficher();
			}	
#endif				
			//idx of the line
		}
		
	}
	
	
	float angleComputationHistogram()
	{
		//TODO:
		return 0.0f;
	}
	
	float angleComputationHAYAI()
	{
		float sum_num = 0.0f;
		float sum_denom = 0.0f;
		
//more robust to define it, see upthere.
//TODO : estimate if an interval of points could be interesting...
#ifndef doubleMatchesOnlyHAYAI		
		for(int i=nbrPoints[0];i--;)	
		{			
			sum_denom += pointsCartesianPose[0][i].get(1,1)*pointsCartesianPose[1][ matches[0].get(1,i+1)-1 ].get(1,1) + pointsCartesianPose[0][i].get(2,1)*pointsCartesianPose[1][ matches[0].get(1,i+1)-1 ].get(2,1) ;
		}
		
		for(int i=nbrPoints[0];i--;)
		{
			sum_num += pointsCartesianPose[0][i].get(1,1)*pointsCartesianPose[1][ matches[0].get(1,i+1)-1 ].get(2,1) - pointsCartesianPose[0][i].get(2,1)*pointsCartesianPose[1][ matches[0].get(1,i+1)-1 ].get(1,1) ;
		}
#else
		for(int i=nbrPoints[0];i--;)	
		{
			//we want to assure that the pair is recognize on both scans :
			if(i+1 == matches[1].get(1, matches[0].get(1,i+1)) )
			{
				sum_denom += pointsCartesianPose[0][i].get(1,1)*pointsCartesianPose[1][ matches[0].get(1,i+1)-1 ].get(1,1) + pointsCartesianPose[0][i].get(2,1)*pointsCartesianPose[1][ matches[0].get(1,i+1)-1 ].get(2,1) ;
			}
		}
		
		for(int i=nbrPoints[0];i--;)
		{
			//we want to assure that the pair is recognize on both scans : idem
			if(i+1 == matches[1].get(1, matches[0].get(1,i+1)) )
			{
				sum_num += pointsCartesianPose[0][i].get(1,1)*pointsCartesianPose[1][ matches[0].get(1,i+1)-1 ].get(2,1) - pointsCartesianPose[0][i].get(2,1)*pointsCartesianPose[1][ matches[0].get(1,i+1)-1 ].get(1,1) ;
			}
		}
#endif
		
		float coeff_zarbi = 1.0f;
		//historical event...
		float theta = coeff_zarbi*atan2( sum_num, sum_denom);		
		
		return theta;
		
	}
	
	
	void computeMatchingCovariance(const Mat<float>& R)
	{		
		Mat<float> tR(transpose(R));
		
#ifdef robustify		
		float meanxx = 1e-4;
		float meanyy = 1e-4;
#endif
		
		for(int m=2;m--;)
		{
			matchP[m].clear();
			matchPinv[m].clear();
			
			for(int k=nbrPoints[m];k--;)
			{
				Mat<float> tP(noiseP[m][k]+corrP[m][k]);
				tP += (R * noiseP[1-m][matches[m].get(1,k+1)-1]) * tR;
				
				
#ifdef robustify			
				for(int i=tP.getLine();i--;)
				{
					if( fabs_(tP.get(i,i)) < meanxx)
						tP.set(meanxx, i,i);
				}
#endif				
				matchP[m].insert(matchP[m].begin(), tP);
				matchPinv[m].insert(matchPinv[m].begin(), invGJ(tP));
			}
		}
	}
	
	
	
	void estimateDeltaPose(float theta_)
	{
		
#ifdef debug_lvl1
		clock_t timerEst = clock();
#endif		
		
		float theta = theta_;
		//TODO : register the computed values to re-use them...
		//TODO : test for the treshold-filtered scheme :
#ifdef BOX		
		float MaxLinear = 0.6f;
#else
		float MaxLinear = 0.2f;
#endif		
		float MaxAngular = 1.7f;
		
		Mat<float> dP(2,1);
		//Mat<float> dPUW(2,1);
		//int nbrpointsused = 0;
		float dtheta = 0.0f;
		Mat<float> deltaP(0.0f, 2,1);
		
		//Mat<float> Pp(2,2);
		Mat<float> sumDiffPose(2,1);
		//Mat<float> sumDiffPoseUW(2,1);
		Mat<float> R(rot2D(theta));
		
		
		float end_criterion = 1e-3;
		float end_criterion_theta = 5e-3;
		
		for(int it=nbrIt;it--;)
		{
			qk.clear();
			Pp = Mat<float>(0.0f,2,2);
			for(int k=matches[0].getColumn();k--;)
			{	
#ifndef		doubleMatchOnlyEstimation						
				Pp += matchPinv[0][k];
#else
				if( matches[1].get( 1, (matches[0].get(1,k+1)-1) +1) == k+1)
				{
					Pp += matchPinv[0][k];
				}
#endif				
				
			}
						
			Pp = invGJ(Pp);
			
			PDelta = Pp;
			
			sumDiffPose = Mat<float>(0.0f,2,1);
			//sumDiffPoseUW = Mat<float>(0.0f,2,1);
			Mat<float> residual(2,1);
			for(int k=matches[0].getColumn();k--;)
			{
#ifndef		doubleMatchOnlyEstimation	
				qk.insert(qk.begin(), R*pointsCartesianPose[1][matches[0].get(1,k+1)-1]);		
				sumDiffPose += (matchPinv[0][k] * (pointsCartesianPose[0][k] - qk[0]) );
#else
				if( matches[1].get( 1, (matches[0].get(1,k+1)-1) +1) == k+1)
				{
					//nbrpointsused++;
					qk.insert(qk.begin(), R*pointsCartesianPose[1][matches[0].get(1,k+1)-1] );			
					residual = (pointsCartesianPose[0][k] - qk[0]);
					sumDiffPose += (matchPinv[0][k] * residual );
					//sumDiffPoseUW += residual;
					//ROS_INFO("MATCHED POINTS...");
				}
#endif
			}
			
			
			
			ROS_INFO("WIDL::LOG DP : ");
			dP = Pp*sumDiffPose;
			dP.afficher();
			ROS_INFO("WIDL::LOG Pp : ");
			Pp.afficher();
			ROS_INFO("WIDL::LOG SumDiffPose : ");
			sumDiffPose.afficher();
			
			//----------------------------------------
			//----------------------------------------
			//		Debug purpose : weights or sum ?
			//----------------------------------------
			//----------------------------------------
			//	dP=dPUW;		//it showed us that the weights are not right since without those the estimation is of a better scale.
			//----------------------------------------
			//----------------------------------------
			/*
			
			ROS_INFO("WIDL::LOG DP UW: ");
			dPUW = (1.0f/nbrpointsused)*sumDiffPoseUW;
			dPUW.afficher();			
			ROS_INFO("WIDL::LOG SumDiffPose UW : ");
			sumDiffPoseUW.afficher();
			//dP=dPUW;
			*/
			//----------------------------------------
			//----------------------------------------
			//----------------------------------------
			//----------------------------------------
			
			//--------------------------
			//NAN ?
			if(isnanM(dP))
			{
				ROS_INFO("WIDL::WARNING : NAN FOUND...");
				dP.afficher();
				dP = Mat<float>(0.0f,2,1);
			}
			//-------------------------
			
			
			//------------------------------------
			//		Treshold-filtered scheme :
			//------------------------------------
			if(norme2(dP) > MaxLinear)
			{
				ROS_INFO("WIDL::WARNING : LINEAR value tresholded : dP :");
				transpose(dP).afficher();
				dP *= 0.0f;
			}
			//-----------------------------------
			//-----------------------------------
			deltaP += dP;
			
			ROS_INFO("WIDL::Estimation dP : it = %d :", nbrIt-it);
			transpose(deltaP).afficher();
			ROS_INFO("WIDL::Estimation COVARIANCE : it = %d :", nbrIt-it);
			transpose(Pp).afficher();
			
			//------------------------------------	
			//------------------------------------
			//END CRITERION :
			ROS_INFO("NORME dP = %f / end criterion = %f.", norme2(dP), end_criterion);
			if(norme2(dP) <= end_criterion)
				it = 0;
			//------------------------------------	
			//------------------------------------
			/*
			if(false && it != 0)	// indeed it has not to be done because it is being taken care of during the computation..
			{
				//we have to move all the points of the previous  scan and rematch them again... :
				//TODO : debug...
#ifdef optimDP
				Mat<float> dPolarPose(2,1);
				for(int k=nbrPoints[1];k--;)
				{
					//let's move them :
					pointsCartesianPose[1][k] -= dP;
					pointsPolarPose[1][k].set( sqrt( pow(pointsCartesianPose[1][k].get(1,1), 2) + pow(pointsCartesianPose[1][k].get(2,1), 2) ), 1,1);
					//pointsPolarPose[1][k].set( pointsPolarPose[1][k].get(2,1)-(-dtheta), 2,1);  
					pointsPolarPose[1][k].set( atan21(pointsCartesianPose[1][k].get(2,1), pointsCartesianPose[1][k].get(1,1)), 2,1);  
				}
				
				//let's recompute their corresponding noise variances :
				prepareUncertainty(noiseP[1], corrP[1], pointsCartesianPose[1],pointsPolarPose[1], delta[1],incidenceAngles[1]);
				//let's rematch them again :
				//--------------------------------
				//		slamProcess : computation : distance Matrix
				//-------------------------------
				computeDistanceMatrix();
				
				//--------------------------------
				//		slamProcess : computation : points matching vectors
				//-------------------------------
				pointsMatching();
				
				//let's recompute the corresponding covariance matrixes :
	#ifndef optimHAYAIinit1				
				theta += 0.0f;
	#else
				theta += angleComputationHAYAI();
	#endif
				computeMatchingCovariance( rot2D(theta) );
#else
				//or : we could only do, assuming that the change is small...
				computeMatchingCovariance(rot2D(theta));
#endif
			}
			*/
			
			//-----------------------------------
			//-----------------------------------
			//
			//		Iteration estimation of theta :
			//
			//-----------------------------------
			//-----------------------------------	
			
#ifndef		iterativeHAYAI
			float num;
			float denom;
			
			for(int iterationTheta = 1;iterationTheta--;)
			{		
				num =0.0f;
				denom = 0.0f;
	
				for(int k=matches[0].getColumn();k--;)
				{
		#ifndef		doubleMatchOnlyEstimation				
					num += (transpose( pointsCartesianPose[0][k] - deltaP - R*pointsCartesianPose[1][matches[0].get(1,k+1)-1] ) * (matchPinv[0][k] * (J * (R * pointsCartesianPose[1][matches[0].get(1,k+1)-1]) ) ) ).get(1,1);
		#else
					if( matches[1].get( 1, (matches[0].get(1,k+1)-1) +1) == k+1)
					{
						num += (transpose( pointsCartesianPose[0][k] - deltaP - R*pointsCartesianPose[1][matches[0].get(1,k+1)-1] ) * (matchPinv[0][k] * (J * (R * pointsCartesianPose[1][matches[0].get(1,k+1)-1]) ) ) ).get(1,1);
					}
		#endif	
				}
	
				for(int k=matches[0].getColumn();k--;)
				{				
		#ifndef		doubleMatchOnlyEstimation				
					denom += (transpose( R*pointsCartesianPose[1][matches[0].get(1,k+1)-1] ) * (J * (matchPinv[0][k] * (J * (R * pointsCartesianPose[1][matches[0].get(1,k+1)-1]) ) ) ) ).get(1,1);
		#else
					if( matches[1].get( 1, (matches[0].get(1,k+1)-1) +1) == k+1)
					{
						denom += (transpose( R*pointsCartesianPose[1][matches[0].get(1,k+1)-1] ) * (J * (matchPinv[0][k] * (J * (R * pointsCartesianPose[1][matches[0].get(1,k+1)-1]) ) ) ) ).get(1,1);
					}
		#endif	
				}
	
				if(denom == 0.0f)
					denom = numeric_limits<float>::epsilon();
		
				dtheta = -num/denom;

				//--------------------------
				//NAN ?
				if(isnan(dtheta))
					dtheta = 0.0f;
				//-------------------------
	
				//------------------------------------
				//		Treshold-filtered scheme :
				//------------------------------------
				if(abs(dtheta) > MaxAngular)
				{
					ROS_INFO("WIDL::WARNING : angular value tresholded (dtheta = %f / max = %f).", dtheta, MaxAngular); 			
					dtheta = 0.0f;
				}
				//-----------------------------------
				//-----------------------------------
	
				theta -= dtheta;
				//R = rot2D(dtheta);
				
				//------------------------------------	
				//------------------------------------
				//END CRITERION :
				ROS_INFO("dTheta = %f / end criterion = %f.", dtheta, end_criterion_theta);
				if( fabs_(dtheta) <= end_criterion_theta)
					iterationTheta = 0;
				//------------------------------------	
				//------------------------------------
		
				ROS_INFO("WIDL::Estimation dTheta : it = %d : dTheta = %f.", nbrIt-it, theta);
		
		
				//-------------------------------------
				//-------------------------------------
#ifndef affineDeplacementAfterRotation				
				if(iterationTheta != 0)
#else
				if(true)	// we have to move the point before we try to reestimate the linear depalcement.
#endif								
				{
					//we have to move all the points of the previous  scan and rematch them again... :
					//TODO : debug...
	#ifdef optimDP
					Mat<float> dPolarPose(2,1);
					for(int k=nbrPoints[0];k--;)
					{
						//let's move them according the estimated parameter.
						//pointsCartesianPose[1][k] -= dP;
						//pointsPolarPose[1][k].set( sqrt( pow(pointsCartesianPose[1][k].get(1,1), 2) + pow(pointsCartesianPose[1][k].get(2,1), 2) ), 1,1);
						pointsPolarPose[0][k].set( pointsPolarPose[0][k].get(2,1)-(-dtheta), 2,1);  
						pointsCartesianPose[0][k].set( pointsPolarPose[0][k].get(1,1)*cos(pointsPolarPose[0][k].get(2,1)), 1,1);
						pointsCartesianPose[0][k].set( pointsPolarPose[0][k].get(1,1)*sin(pointsPolarPose[0][k].get(2,1)), 2,1);
					}
		
					//let's recompute their corresponding noise variances :
					prepareUncertainty(noiseP[0], corrP[0], pointsCartesianPose[0],pointsPolarPose[0], delta[0],incidenceAngles[0]);
					//let's rematch them again :
					//--------------------------------
					//		slamProcess : computation : distance Matrix
					//-------------------------------
					computeDistanceMatrix();
		
					//--------------------------------
					//		slamProcess : computation : points matching vectors
					//-------------------------------
					pointsMatching();
					
					//redoing an initial guess given the new points positions and points matching :
					dtheta = angleComputationHAYAI();
					R = rot2D(theta);
					//let's recompute the corresponding covariance matrixes :
					computeMatchingCovariance( R );
	#else
					//or : we could only do, assuming that the change is small...
					//R = rot2D(dtheta);
					computeMatchingCovariance( rot2D(theta) );
	#endif
				}
			}

#else
			dtheta = angleComputationHAYAI();
	
			//--------------------------
			//NAN ?
			if(isnan(dtheta))
				dtheta = 0.0f;
			//-------------------------
	
			//------------------------------------
			//		Treshold-filtered scheme :
			//------------------------------------
			if(abs(dtheta) > MaxAngular)
			{
				ROS_INFO("WIDL::WARNING : angular value tresholded (dtheta = %f / max = %f).", dtheta, MaxAngular); 			
				dtheta = 0.0f;
			}
			//-----------------------------------
			//-----------------------------------
	
			theta += dtheta;
#endif
						
			
		}

		R = rot2D(theta);
		
#ifdef affineDeplacementAfterRotation		
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
//		REESTIMATION OF THE LINEAR DEPLACEMENT 
//		NOW THAT WE HAVE RETREIVED THE ANGULAR DEPLACEMENT
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
		if(fabs_(theta) >= sqrt(varnoise_obs_theta))
		{
			qk.clear();
			Pp = Mat<float>(0.0f,2,2);
			for(int k=matches[0].getColumn();k--;)
			{	
	#ifndef		doubleMatchOnlyEstimation						
				Pp += matchPinv[0][k];
	#else
				if( matches[1].get( 1, (matches[0].get(1,k+1)-1) +1) == k+1)
				{
					Pp += matchPinv[0][k];
				}
	#endif				
			
			}
					
			Pp = invGJ(Pp);
		
			PDelta = Pp;
		
			sumDiffPose = Mat<float>(0.0f,2,1);
			//sumDiffPoseUW = Mat<float>(0.0f,2,1);
			Mat<float> residual(2,1);
			for(int k=matches[0].getColumn();k--;)
			{
	#ifndef		doubleMatchOnlyEstimation	
				qk.insert(qk.begin(), R*pointsCartesianPose[1][matches[0].get(1,k+1)-1]);		
				sumDiffPose += (matchPinv[0][k] * (pointsCartesianPose[0][k] - qk[0]) );
	#else
				if( matches[1].get( 1, (matches[0].get(1,k+1)-1) +1) == k+1)
				{
					//nbrpointsused++;
					qk.insert(qk.begin(), R*pointsCartesianPose[1][matches[0].get(1,k+1)-1] );			
					residual = (pointsCartesianPose[0][k] - qk[0]);
					sumDiffPose += (matchPinv[0][k] * residual );
					//sumDiffPoseUW += residual;
					//ROS_INFO("MATCHED POINTS...");
				}
	#endif
			}
		
		
			/*
			ROS_INFO("WIDL::LOG DP : ");
			dP = Pp*sumDiffPose;
			dP.afficher();
			ROS_INFO("WIDL::LOG Pp : ");
			Pp.afficher();
			ROS_INFO("WIDL::LOG SumDiffPose : ");
			sumDiffPose.afficher();
			*/
			//--------------------------
			//NAN ?
			if(isnanM(dP))
			{
				ROS_INFO("WIDL::AFFINE WARNING : NAN FOUND...");
				dP.afficher();
				dP = Mat<float>(0.0f,2,1);
			}
			//-------------------------
		
		
			//------------------------------------
			//		Treshold-filtered scheme :
			//------------------------------------
			if(norme2(dP) > MaxLinear)
			{
				ROS_INFO("WIDL::AFFINE WARNING : LINEAR value tresholded : dP :");
				transpose(dP).afficher();
				dP *= 0.0f;
			}
			//-----------------------------------
			//-----------------------------------
			deltaP = dP;
		
			ROS_INFO("WIDL::AFFINE Estimation dP :");
			transpose(deltaP).afficher();
			ROS_INFO("WIDL::AFFINE Estimation COVARIANCE :");
			transpose(Pp).afficher();
		}
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
#endif
		
		computeEstimationCovariance(R);

#ifdef	LOWPASS_FILTER
		float tresh = 0.4f;
		if( fabs_(deltaP.get(1,1)-deltaPose.x) > tresh)
			deltaP.set(0.0f,1,1);
		if( fabs_(deltaP.get(2,1)-deltaPose.y) > tresh)
			deltaP.set(0.0f,2,1);
#endif		
				
		//deltaPose.x = -2*deltaP.get(1,1);
		deltaPose.x = -deltaP.get(1,1);
		//deltaPose.y = -2*deltaP.get(2,1);
		deltaPose.y = -deltaP.get(2,1);
		deltaPose.theta = theta;
		
		ROS_INFO("DELTA POSE : x = %f ; y = %f ; theta = %f.", deltaPose.x, deltaPose.y, deltaPose.theta);
		
		
		
#ifdef debug_lvl1
		ROS_INFO("WIDL::EXECUTION Estimation : %f Hz.", (float)((float)CLOCKS_PER_SEC/(clock()-timerEst)) );
#endif				
		
	}
	
	void computeEstimationCovariance(const Mat<float>& R)
	{
		rT = 0.0f;
		PpTheta = Mat<float>(0.0f,2,1);
				
		for(int k=matches[0].getColumn();k--;)
		{	
#ifndef		doubleMatchOnlyEstimation
			Mat<float> temp(R*pointsCartesianPose[1][matches[0].get(1,k+1)-1]);
			rT -= ( transpose(temp)*( J * matchPinv[0][k] * J )* temp ).get(1,1);
			PpTheta += (matchPinv[0][k] * J) * temp;
#else
			if( matches[1].get( 1, (matches[0].get(1,k+1)-1) +1) == k+1)
			{
				Mat<float> temp(R*pointsCartesianPose[1][matches[0].get(1,k+1)-1]);
				rT -= ( transpose(temp)*( J * matchPinv[0][k] * J )* temp ).get(1,1);
				PpTheta += (matchPinv[0][k] * J) * temp;
			}
#endif			
		}
		
		if(isnan(rT))
			rT = numeric_limits<float>::epsilon();
		if(rT == 0.0f)
			rT = numeric_limits<float>::epsilon();
		
		ROS_INFO("WIDL::RT = %f", rT);
		//------------------------------------------
		//------------------------------------------
		//------------------------------------------
		PpTheta = (1.0f/rT) * (Pp * PpTheta);
		
		ROS_INFO("WIDL::Pest : ");
		
		Pest = operatorC( operatorL( Pp, PpTheta), operatorL( transpose(PpTheta), Mat<float>(1.0f/rT, 1,1) ) );
		Pest.afficher();
	
	}
	
	
	Mat<float> rot2D(float angle)
	{
		Mat<float> r(2,2);
		r.set(cos(angle), 1,1);
		r.set(cos(angle), 2,2);
		r.set(sin(angle), 1,2);
		r.set(-sin(angle), 2,1);
		return r;
	}
	
};

int main(int argc, char* argv[])
{

	ros::init(argc,argv,"WIDL_SLAM");
	
	int rate = 100;
	int nbrIt = 1;
	if(argc>1)
		nbrIt = atoi(argv[1]);
	if(argc>2)
		rate = atoi(argv[2]);
		
#ifdef OCCUPANCY_GRID_USE
	float precX = 0.05f;
	float precY = 0.05f;
	
	if(argc>3)
		precX = atof(argv[3]);
	if(argc>4)
		precY = atof(argv[4]);
	WIDL instanceHS(nbrIt,rate,precX,precY);	
#else	
	WIDL instanceHS(nbrIt,rate);
#endif	
	
	return 0;
}


	
