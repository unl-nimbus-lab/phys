#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include "MATv2/Mat.h"
#include "EKF/EKF.h"

//#define debug_lvl1
//#define debug_lvl2
#define debug_lvl3	
//#define debug_lvl4	
#define debug_lvl5	// bug wrong values of the smooth_ranges in the feature extraction function.
#define debug_lvl6	//covar estimation on delta_pose.

#define INF 1.0f/numeric_limits<float>::epsilon()
//#define PI 3.1415926535f
#define RANGE_HEUR n

#define centroidDoubleMatchesUse

enum fType { MAXF, MINF, INFLF};

class HAYAI_SLAM
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
	ros::Subscriber smooth_scan_sub;
	
	//DATA :
	geometry_msgs::Pose2D deltaPose;
	geometry_msgs::Pose2D globalPose;
	nav_msgs::Path path;
	nav_msgs::Path EKFpath;
	
	int scan_queue_size;	//default : 2.
	
	vector<sensor_msgs::LaserScan> ups_scans;
	vector<sensor_msgs::LaserScan> smooth_scans;
	
	//New scans that are being received are stored in those std::vector in wait of being use.
	
	int nbrScansReceived;
	bool newScansReceived;
	bool newUpsScanReceived;
	bool newSmoothScanReceived;	
	sensor_msgs::LaserScan ups_scansInUse[2];
	sensor_msgs::LaserScan smooth_scansInUse[2];
	
	//handlers for the scans that are being used currently.
	
	int* nbrFeatures;
	vector<Mat<float> > featuresCartesianPose[2];
	vector<Mat<float> > featuresCartesianPoseCentered[2];
	vector<Mat<float> > featuresPolarPose[2];
	vector<fType> featuresType[2];
	
	Mat<float> distMat;
	Mat<float> statsSLAM;
	Mat<float> stats;
	Mat<int> matches[2];
	Mat<float> centroids[2];
	
	int n;	//nbr of range values...
	
	//------------------------------------------
	//-----------------------------------------
	//EEKF :
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
	
	//------------------------------------------
	//-----------------------------------------
	
	
	public :
	
	HAYAI_SLAM(int rate_ = 100, int scan_queue_size_ = 2 ) : rate(rate), scan_queue_size(scan_queue_size_)
	{
		ups_scan_sub = nh.subscribe("/scan_extmedian_filter", 10, &HAYAI_SLAM::callback_ups, this);
		smooth_scan_sub = nh.subscribe("/scan_smooth_filter", 10, &HAYAI_SLAM::callback_smooth, this); 
	
		nbrFeatures = new int[scan_queue_size];
		deltaPose_pub = nh.advertise<geometry_msgs::Pose2D>("HAYAY_SLAM/DeltaPose", 10);
		globalPose_pub = nh.advertise<geometry_msgs::Pose2D>("HAYAY_SLAM/Pose", 10);
		path_pub = nh.advertise<nav_msgs::Path>("HAYAI_SLAM/Path",10);
		pose_stamped_pub = nh.advertise<geometry_msgs::PoseStamped>("HAYAI_SLAM/POSESTAMPED",10);
		EKFpath_pub = nh.advertise<nav_msgs::Path>("HAYAI_SLAM/EKFPath",10);
		EKFpose_stamped_pub = nh.advertise<geometry_msgs::PoseStamped>("HAYAI_SLAM/EKFPOSESTAMPED",10);
		
		nbrScansReceived = 0;
		newScansReceived = false;
		newUpsScanReceived = false;
		newSmoothScanReceived = false;
		
		globalPose.x = 0.0f;
		globalPose.y = 0.0f;
		globalPose.theta = 0.0f;
		
		n=0;
		centroids[0] = Mat<float>(2,1);
		centroids[1] = Mat<float>(2,1);
		
		//--------------------------------------------------------
		//--------------------------------------------------------
		//EKF :
		nbrstate = 6;
		nbrcontrol = 0;
		nbrobs = 3;
		dt = 1;
		stdnoise = 1e-5;
		stdnoise_obs = 1e-4;
		ext = false;
		filteron = true;
		noise = false;

		EKFPose = Mat<float>((float)0,nbrstate,1);  
		EKFPoseCovar = Mat<float>((float)0,nbrstate,nbrstate);  
		instanceEEKF = new EEKF<float>(nbrstate,nbrcontrol,nbrobs,dt,stdnoise,EKFPose,ext,filteron,noise);

		Mat<float> A((float)0,nbrstate,nbrstate);
		for(int i=1;i<=nbrstate/2;i++)	A.set((float)1,i,i);
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
		for(int i=1;i<=nbrstate;i++)	Q.set( stdnoise*stdnoise, i,i);
		//Q.set( pow(stdnoise_obs*(1e1),2), 3,3);		//allow more variations in theta since the observer is to be trusted.
		//Q.set( stdnoise_obs*1e-4, 3,3);
		//Q.set( pow(stdnoise_obs*1e-1,2), 2,2);		//account for more uncertainty in the y value since it is more flawed than the others...
		Q.afficher();
		instanceEEKF->initQ(Q);
		
		R = Mat<float>(0.0f,nbrobs,nbrobs);
		//for(int i=1;i<=nbrobs;i++)	R.set( (i==nbrobs? stdnoise*1e-2 : stdnoise_obs*stdnoise_obs), i,i);
		R.set( pow(stdnoise*2, 2), 1,1);
		R.set( pow(stdnoise*2,2), 2,2);	//more variations on y...
		R.set( pow(stdnoise*1e-2, 2), 3,3);			//less variations on theta...
		R.afficher();
		instanceEEKF->initR(R);
		
		
		//--------------------------------------------------------
		//--------------------------------------------------------
		
		
		this->mainLoop();
	}
	
	~HAYAI_SLAM()
	{
		delete[] nbrFeatures;
		delete instanceEEKF;
	}
	
	void callback_ups(sensor_msgs::LaserScan ups_scan)
	{
		ups_scans.insert( ups_scans.begin(), ups_scan);
		
		if(ups_scans.size() > scan_queue_size)
			ups_scans.pop_back();
			
		newUpsScanReceived = true;
		nbrScansReceived++;
		
		if(newSmoothScanReceived)
			newScansReceived = true;
	}
	
	void callback_smooth(sensor_msgs::LaserScan smooth_scan)
	{
		smooth_scans.insert( smooth_scans.begin(), smooth_scan);
		
		if(smooth_scans.size() > scan_queue_size)
			smooth_scans.pop_back();
			
		newSmoothScanReceived = true;
		nbrScansReceived++;
		
		if(newUpsScanReceived)
			newScansReceived = true;
	}
	
	void mainLoop()
	{
		int count_info = 100;
		
#ifdef debug_lvl6		
		//Gestion ecriture dans un fichier :
		string filepath("/home/kevidena/ROS/sandbox/HAYAI_SLAM/src/log.txt");
		FILE* log = fopen(filepath.c_str(), "w+");
		if(log == NULL)
		{
			cout << "ERROR : cannot open the file LOG." << endl;
			exit(1);
		}
		else
			cout << "File opened LOG." << endl;
#endif
	
		while(nbrScansReceived < 2)
		{
			if(count_info>100)
			{
				ROS_INFO("HAYAI_SLAM::Initialization : ...");
				count_info = 0;
			}
			count_info++;
			
			ros::spinOnce();
		}
		
		//initialize the first pair of scans that are to be used :
		ups_scansInUse[0] = ups_scans[0];
		smooth_scansInUse[0] = smooth_scans[0];
		
		//initialize the first features extracted from the first pair of scans :
		nbrFeatures[0] = extractFeatures(ups_scansInUse[0], smooth_scansInUse[0], featuresCartesianPose[0], featuresPolarPose[0], featuresType[0]);
		computeCentroids();
		
		ROS_INFO("HAYAI_SLAM::Initialization : DONE.");
		
		while(nbrScansReceived < 4)
		{
			if(count_info>100)
			{
				ROS_INFO("HAYAI_SLAM::Waiting for scans in order to begin the loop ...");
				count_info = 0;
			}
			count_info++;
			
			ros::spinOnce();
		}
		
#ifdef debug_lvl2	
		//Gestion ecriture dans un fichier :
		string filepath("./log.txt");
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
			s << smooth_scans[0].ranges[i];
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
				newUpsScanReceived = false;
				newSmoothScanReceived = false;
				newScansReceived = false;
				
				//--------------------------------
				//--------------------------------
				
				
				//--------------------------------
				//		slamProcess : initialization
				//--------------------------------
				clock_t timer_init = clock();
				
				ups_scansInUse[1] = ups_scansInUse[0];
				smooth_scansInUse[1] = smooth_scansInUse[1];
				
				nbrFeatures[1] = nbrFeatures[0];
				featuresCartesianPose[1] = featuresCartesianPose[0];
				featuresCartesianPoseCentered[1] = featuresCartesianPoseCentered[0];
				featuresPolarPose[1] = featuresPolarPose[0];
				featuresType[1] = featuresType[0];
				centroids[1] = centroids[0];
				
				//---------------------------------------

#ifdef debug_lvl1
				ROS_INFO("HAYAI_SLAM::EXECUTION INIT : %f Hz.", (float)(1.0/((float)(clock()-timer_init)/CLOCKS_PER_SEC)) );
				timer_init = clock();
#endif
				//initialize the first pair of scans that are to be used :
				ups_scansInUse[0] = ups_scans[0];
				smooth_scansInUse[0] = smooth_scans[0];
		
				//initialize the first features extracted from the first pair of scans :
				nbrFeatures[0] = extractFeatures(ups_scansInUse[0], smooth_scansInUse[0], featuresCartesianPose[0], featuresPolarPose[0], featuresType[0]);
				



								
				
#ifdef debug_lvl1				
				ROS_INFO("HAYAI_SLAM::EXECUTION INITFEATURES : %f Hz.", (float)(1.0/((float)(clock()-timer_init)/CLOCKS_PER_SEC)) );
#endif				
				//------------------------------------------------------------------------------------------------
				//------------------------------------------------------------------------------------------------
				
				if(nbrFeatures[0] != 0 && nbrFeatures[1] != 0)
				{
#ifdef debug_lvl1				
					ROS_INFO(" NBR FEATURES : %d * %d = %d.", nbrFeatures[0],nbrFeatures[1], nbrFeatures[0]*nbrFeatures[1]);
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
					ROS_INFO("HAYAI_SLAM::EXECUTION DITMAT : %f Hz.", (float)(1.0/((float)(clock()-timer_mat)/CLOCKS_PER_SEC)) );
					clock_t timer_matching = clock();
#endif				

					//--------------------------------
					//		slamProcess : computation : features matching vectors
					//-------------------------------
					featuresMatching();
					
#ifdef debug_lvl1					
					ROS_INFO("HAYAI_SLAM::EXECUTION MATCHING : %f Hz.", (float)(1.0/((float)(clock()-timer_matching)/CLOCKS_PER_SEC)) );
#endif
					
					//--------------------------------
					//		slamProcess : computation : centroid computation
					//-------------------------------
					
#ifndef	centroidDoubleMatchesUse					
					computeCentroids();
#else
					computeCentroidsDoubleMatches();
#endif
					estimateDeltaPose();
					
					
#ifdef debug_lvl1
					ROS_INFO("HAYAI_SLAM::slamProcess : distances stats : mean = %f ; var = %f.", stats.get(1,1), stats.get(2,1) );
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
					ROS_INFO("HAYAI_SLAM::slamProcess : not enough features ...");
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
				//---------
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
				//--------------------
				instanceEEKF->measurement_Callback(updatePose);
				//instanceEEKF->setCommand(updatePose);
    			instanceEEKF->state_Callback();
    			EKFPose = instanceEEKF->getX();
    			EKFPoseCovar = instanceEEKF->getSigma();
    			
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
				
				ROS_INFO("GLOBAL POSE : x = %f ; y = %f ; theta = %f", globalPose.x, globalPose.y, globalPose.theta);
				
				std_msgs::Header head_path;
				head_path.seq = nbrScansReceived/4;
				head_path.frame_id = "map";
				
				geometry_msgs::Pose path_pose;
				path_pose.position.x = globalPose.x;
				path_pose.position.y = globalPose.y;
				path_pose.position.z = (float)0;
				
				
				path_pose.orientation.x = (float)0;
				path_pose.orientation.y = (float)0;
				path_pose.orientation.z = globalPose.theta;
				path_pose.orientation.w = (float)1;
				
				
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
				
				
				path_pose.orientation.x = (float)0;
				path_pose.orientation.y = (float)0;
				path_pose.orientation.z = EKFPose.get(3,1);
				path_pose.orientation.w = (float)1;
				
				
				path_poseStamped.header.stamp = ros::Time(0);
				path_poseStamped.header.frame_id = "map";
				path_poseStamped.pose = path_pose;
				
				EKFpath.poses.push_back(path_poseStamped);
				EKFpath.header.stamp = ros::Time(0);
				EKFpath.header.frame_id = "map";
				
				EKFpath_pub.publish(EKFpath);
				EKFpose_stamped_pub.publish(path_poseStamped);
				
				//-------------------------------
				//		STATS
				//-------------------------------
				
				statsSLAM.set(statsSLAM.get(1,1)+ 0, 1,1);//deltaPose.y;
				statsSLAM.set(statsSLAM.get(2,1) + pow( deltaPose.x, 2), 2,1);
				
				statsSLAM.set(statsSLAM.get(1,2)+ 0, 1,2);//deltaPose.y;
				statsSLAM.set(statsSLAM.get(2,2) + pow( deltaPose.y, 2), 2,2);
				
				statsSLAM.set(statsSLAM.get(1,3)+ 0, 1,3);//deltaPose.y;
				statsSLAM.set(statsSLAM.get(2,3) + pow( deltaPose.theta, 2), 2,3);
				
				ROS_INFO("STATS SLAM : nbrIt : %d ; STATS :", nbrScansReceived/4);
				((1.0f/(nbrScansReceived/4 - 1))*statsSLAM).afficher();
				
				//-------------------------------------
				//-------------------------------------
					
				ROS_INFO("HAYAI_SLAM::EXECUTION : %f Hz.", (float)(1.0/((float)(clock()-timer)/CLOCKS_PER_SEC)) );	
				
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
				string filepath0("/home/kevidena/ROS/sandbox/HAYAI_SLAM/src/log0.txt");
				FILE* log0 = fopen(filepath0.c_str(), "w+");
				if(log0 == NULL)
				{
					cout << "ERROR : cannot open the file LOG0." << endl;
					exit(1);
				}
				else
					cout << "File opened LOG0." << endl;
			
				string filepath1("/home/kevidena/ROS/sandbox/HAYAI_SLAM/src/log1.txt");
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
						
				for(int i=0;i<nbrFeatures[0];i++)	
				{
					stringstream s;
					s << featuresCartesianPose[0][i].get(1,1) << " , " << featuresCartesianPose[0][i].get(2,1) << " , " << matches[0].get(1,i+1);
					s << endl;
					//cout << s.str();
					fputs( s.str().c_str(), log0);
				}
		
				for(int i=0;i<nbrFeatures[1];i++)	
				{
					stringstream s;
					s << featuresCartesianPose[1][i].get(1,1) << " , " << featuresCartesianPose[1][i].get(2,1) << " , " << matches[1].get(1,i+1);
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
	
	int extractFeatures(const sensor_msgs::LaserScan& ups_scan, const sensor_msgs::LaserScan& smooth_scan, vector<Mat<float> >& featuresCartesianPose_, vector<Mat<float> >& featuresPolarPose_, vector<fType>& featuresType_)
	{
		featuresCartesianPose_.clear();
		featuresPolarPose_.clear();
		featuresType_.clear();
		
		int nbrfeat = 0;
		
		n = (int)((smooth_scan.angle_max-smooth_scan.angle_min)/smooth_scan.angle_increment);
		float a_inc = ups_scan.angle_increment;
		Mat<float> tempCart(2,1);
		Mat<float> tempPol(2,1);
		
		float offset = smooth_scan.intensities[0];
		float med_size = smooth_scan.intensities[1];
		float ect = 0.0f;
		
		for(int i=n;i--;)	ect += pow(smooth_scan.ranges[i]-offset, 2);
		ect /= n-1;
		ect = sqrt(ect);
		
		int nbr_max = 0;
		int nbr_min = 0;
		int nbr_infl = 0;
		
		for(int i=n;i--;)
		{
			if(i>med_size+1 || i<n-med_size-1)
			{
			
				if(smooth_scan.ranges[i] != offset)
				{
					/*
					if(smooth_scan.ranges[i-1]<smooth_scan.ranges[i])
					{
						if(smooth_scan.ranges[i+1]<smooth_scan.ranges[i])
						{
							nbrfeat++;
							nbr_max++;
							//MAXIMUM :
							featuresType_.insert(featuresType_.begin(), MAXF);
							
							tempPol.set(ups_scan.ranges[i], 1,1);
							tempPol.set(ups_scan.angle_min + i*a_inc, 2,1);
							tempCart.set( tempPol.get(1,1)*cos(tempPol.get(2,1)), 1,1);
							tempCart.set( tempPol.get(1,1)*sin(tempPol.get(2,1)), 2,1);
							
							featuresPolarPose_.insert(featuresPolarPose_.begin(), tempPol);
							featuresCartesianPose_.insert(featuresCartesianPose_.begin(), tempCart);
						}
					}
					else if(smooth_scan.ranges[i-1]>smooth_scan.ranges[i])
					{
						if(smooth_scan.ranges[i+1]>smooth_scan.ranges[i])
						{
							nbrfeat++;
							nbr_min++;
							//MINIMUM :
							featuresType_.insert(featuresType_.begin(), MINF);
							
							tempPol.set(ups_scan.ranges[i], 1,1);
							tempPol.set(ups_scan.angle_min + i*a_inc, 2,1);
							tempCart.set( tempPol.get(1,1)*cos(tempPol.get(2,1)), 1,1);
							tempCart.set( tempPol.get(1,1)*sin(tempPol.get(2,1)), 2,1);
							
							featuresPolarPose_.insert(featuresPolarPose_.begin(), tempPol);
							featuresCartesianPose_.insert(featuresCartesianPose_.begin(), tempCart);
						}
					}
					*/
					
					
					if(smooth_scan.ranges[i] > offset + 2*ect)
					{
							//we know that this point is not due to noise..
							
							if(smooth_scan.ranges[i-1] < smooth_scan.ranges[i] && smooth_scan.ranges[i+1] < smooth_scan.ranges[i])
							{
								//we know that this point is the local max :
#ifdef debug_lvl5								
								cout << " MAX VALEUR theta = " << (smooth_scan.angle_min+i*smooth_scan.angle_increment)*180/PI << "SCAN RANGE  = " << smooth_scan.ranges[i] << " / offset =" << offset << " +ect =" << ect << ";  = " << offset+ect << endl;
#endif								
								nbrfeat++;
								nbr_max++;
								//MAXIMUM :
								featuresType_.insert(featuresType_.begin(), MAXF);
							
								tempPol.set(ups_scan.ranges[i], 1,1);
								tempPol.set(ups_scan.angle_min + i*a_inc, 2,1);
								//tempPol.set((ups_scan.angle_min + i*a_inc)*PI/180, 2,1);
								tempCart.set( tempPol.get(1,1)*cos(tempPol.get(2,1)), 1,1);
								tempCart.set( tempPol.get(1,1)*sin(tempPol.get(2,1)), 2,1);
							
								featuresPolarPose_.insert(featuresPolarPose_.begin(), tempPol);
								featuresCartesianPose_.insert(featuresCartesianPose_.begin(), tempCart);
							}
						
					}
					else if( smooth_scan.ranges[i] < offset - 2*ect)
					{
							//we know that this point is not due to noise..
							
							if(smooth_scan.ranges[i-1] > smooth_scan.ranges[i] && smooth_scan.ranges[i+1] > smooth_scan.ranges[i])
							{
								//we know that this point is the local min :
#ifdef debug_lvl5								
								cout << " MIn VALEUR theta = " << (smooth_scan.angle_min+i*smooth_scan.angle_increment)*180/PI << "SCAN RANGE  = " << smooth_scan.ranges[i] << " / offset =" << offset << " -ect =" << ect << ";  = " << offset-ect << endl;
#endif								
								nbrfeat++;
								nbr_min++;
								//MINIMUM :
								featuresType_.insert(featuresType_.begin(), MINF);
							
								tempPol.set(ups_scan.ranges[i], 1,1);
								tempPol.set(ups_scan.angle_min + i*a_inc, 2,1);
								//tempPol.set((ups_scan.angle_min + i*a_inc)*PI/180, 2,1);
								tempCart.set( tempPol.get(1,1)*cos(tempPol.get(2,1)), 1,1);
								tempCart.set( tempPol.get(1,1)*sin(tempPol.get(2,1)), 2,1);
							
								featuresPolarPose_.insert(featuresPolarPose_.begin(), tempPol);
								featuresCartesianPose_.insert(featuresCartesianPose_.begin(), tempCart);
							}
						
					}
			
				}
				
				/*else if(smooth_scan.ranges[i-1] != smooth_scan.ranges[i+1])
				{
					if( (smooth_scan.ranges[i-1]>offset && smooth_scan.ranges[i+1]<offset) || (smooth_scan.ranges[i+1]>offset && smooth_scan.ranges[i-1]<offset) )
					{
						nbrfeat++;
						nbr_infl++;
						//INFLECTION POINT :
						featuresType_.insert(featuresType_.begin(), INFLF);
						
						tempPol.set(ups_scan.ranges[i], 1,1);
						tempPol.set(ups_scan.angle_min + i*a_inc, 2,1);
						tempCart.set( tempPol.get(1,1)*cos(tempPol.get(2,1)), 1,1);
						tempCart.set( tempPol.get(1,1)*sin(tempPol.get(2,1)), 2,1);
						
						featuresPolarPose_.insert(featuresPolarPose_.begin(), tempPol);
						featuresCartesianPose_.insert(featuresCartesianPose_.begin(), tempCart);
					}
				}
				*/
			}
		
		}
		
		ROS_INFO(" NBR : MAX = %d ; MIN = %d ; INFL = %d.", nbr_max, nbr_min, nbr_infl);
		
		return nbrfeat;
	
	
	}
	
	float distType(const fType& f0, const fType& f1)
	{
		return (f0 == f1? 0.0f : INF);
	}
	
	
	void computeDistanceMatrix()
	{
		distMat = Mat<float>(nbrFeatures[0],nbrFeatures[1]);
		stats = Mat<float>(2,1);
		float dMmean = 0.0f;
		float dMvar = 0.0f;
		
		float w1 = 1.0f;
		float w2 = 1.0f;
		float w3 = 1.0f;
		float w4 = 1.0f;
		
		for(int i=nbrFeatures[0];i--;)
		{
			for(int j=nbrFeatures[1];j--;)
			{
				float dist = sqrt( w1*pow(featuresPolarPose[0][i].get(1,1)-featuresPolarPose[1][j].get(1,1),2) + w2*pow(featuresPolarPose[0][i].get(2,1)-featuresPolarPose[1][j].get(2,1), 2) );
				dist += sqrt( w3*pow(featuresCartesianPose[0][i].get(1,1)-featuresCartesianPose[1][j].get(1,1),2) + w3*pow(featuresCartesianPose[0][i].get(2,1)-featuresCartesianPose[1][j].get(2,1), 2) );
				dist += distType(featuresType[0][i], featuresType[1][j]);
				distMat.set( dist , i+1,j+1);
				
				dMmean += dist;
			}
		}
		
		dMmean /= (nbrFeatures[0]*nbrFeatures[1]);
		for(int i=nbrFeatures[0];i--;)
		{
			for(int j=nbrFeatures[1];j--;)
			{
				dMvar += pow( dMmean-distMat.get(i+1,j+1), 2);
			}
		}
		dMvar /= (nbrFeatures[0]*nbrFeatures[1]) - 1;
		
		stats.set(dMmean, 1,1);
		stats.set(dMvar, 2,1);
	}
	
	void featuresMatching()
	{
		matches[0] = Mat<int>(1,nbrFeatures[0]);
		matches[1] = Mat<int>(1,nbrFeatures[1]);
		
		int range_heur = RANGE_HEUR;
		int idx1 = 0;
		int idx2 = 0;
		
		for(int k=nbrFeatures[0];k--;)
		{
			if(k+1-range_heur <= 1)
				idx1 = 1;
			else
				idx1 = k+1-range_heur;
			
			if(k+1+range_heur >= nbrFeatures[1])
				idx2 = nbrFeatures[1];
			else
				idx2 = k+1+range_heur;
			
			
			matches[0].set( (idx1-1) + idmin( extract(&distMat, k+1, idx1, k+1, idx2) ).get(2,1), 1,k+1);						
			
			
			//idx of the column
		}
		
		for(int k=nbrFeatures[1];k--;)
		{
			if(k+1-range_heur <=1)
				idx1 = 1;
			else
				idx1 = k+1-range_heur;
			
			if(k+1+range_heur >=nbrFeatures[0])
				idx2 = nbrFeatures[0];
			else
				idx2 = k+1+range_heur;
				
			matches[1].set( (idx1-1) + idmin( extract(&distMat, idx1, k+1, idx2, k+1) ).get(1,1), 1,k+1);	
#ifdef debug_lvl4			
			if(k==nbrFeatures[1]/2)
			{
				cout << idx1-1 << endl;
				extract(&distMat, idx1, k+1, idx2, k+1).afficher();
				idmin( extract(&distMat, idx1, k+1, idx2, k+1) ).afficher();
			}	
#endif				
			//idx of the line
		}
		
	}
	
	
	void computeCentroids()
	{
		centroids[0].set(0.0f,1,1);
		centroids[0].set(0.0f,2,1);
		for(int i=nbrFeatures[0];i--;)	centroids[0] += featuresCartesianPose[0][i];
		centroids[0] *= (float)(1.0f/nbrFeatures[0]);
		
		/*
		centroids[1].set(0.0f,1,1);
		centroids[1].set(0.0f,2,1);
		for(int i=nbrFeatures[1];i--;)	centroids[1] += featuresCartesianPose[1][i];
		centroids[1] *= (float)(1.0f/nbrFeatures[1]);
		*/
		
		featuresCartesianPoseCentered[0].clear();
		for(int i=nbrFeatures[0];i--;)	
		{
			featuresCartesianPoseCentered[0].insert(featuresCartesianPoseCentered[0].begin(), featuresCartesianPose[0][i]-centroids[0]);
			//transpose(featuresCartesianPoseCentered[0][nbrFeatures[0]-i-1]).afficher();
			//transpose(featuresCartesianPose[0][i]).afficher();
			//transpose(featuresPolarPose[0][i]).afficher();
		}
		
	}
	
	//TODO:debugging
	void computeCentroidsDoubleMatches()
	{
		centroids[0].set(0.0f,1,1);
		centroids[0].set(0.0f,2,1);
		
		int nbrfeaturesUsed = 0;
		for(int i=nbrFeatures[0];i--;)	
		{
			//we want to assure that the pair is recognize on both scans :
			if(i+1 == matches[1].get(1, matches[0].get(1,i+1)) )
			{
				centroids[0] += featuresCartesianPose[0][i];
				nbrfeaturesUsed++;
			}
		}
		
		//-------------------------------------------
		//			WATCH OUT
		//-------------------------------------------
		if(nbrfeaturesUsed == 0)
			return computeCentroids();
		//-------------------------------------------
		//			WATCH OUT
		//-------------------------------------------			
		
		centroids[0] *= (float)(1.0f/(float)nbrfeaturesUsed);
		
		/*
		centroids[1].set(0.0f,1,1);
		centroids[1].set(0.0f,2,1);
		for(int i=nbrFeatures[1];i--;)	centroids[1] += featuresCartesianPose[1][i];
		centroids[1] *= (float)(1.0f/nbrFeatures[1]);
		*/
		
		featuresCartesianPoseCentered[0].clear();
		for(int i=nbrFeatures[0];i--;)	
		{
			featuresCartesianPoseCentered[0].insert(featuresCartesianPoseCentered[0].begin(), featuresCartesianPose[0][i]-centroids[0]);
			//transpose(featuresCartesianPoseCentered[0][nbrFeatures[0]-i-1]).afficher();
			//transpose(featuresCartesianPose[0][i]).afficher();
			//transpose(featuresPolarPose[0][i]).afficher();
		}
		
	}
	
	
	void estimateDeltaPose()
	{
		/*
		deltaPose.x = 0;
		deltaPose.y = 0;
		deltaPose.theta = 0;
		*/
		
		float sum_num = 0.0f;
		float sum_denom = 0.0f;
		

#ifndef centroidDoubleMatchesUse		
		for(int i=nbrFeatures[0];i--;)	
		{			
			sum_denom += featuresCartesianPoseCentered[0][i].get(1,1)*featuresCartesianPoseCentered[1][ matches[0].get(1,i+1)-1 ].get(1,1) + featuresCartesianPoseCentered[0][i].get(2,1)*featuresCartesianPoseCentered[1][ matches[0].get(1,i+1)-1 ].get(2,1) ;
		}
		
		for(int i=nbrFeatures[0];i--;)
		{
			sum_num += featuresCartesianPoseCentered[0][i].get(1,1)*featuresCartesianPoseCentered[1][ matches[0].get(1,i+1)-1 ].get(2,1) - featuresCartesianPoseCentered[0][i].get(2,1)*featuresCartesianPoseCentered[1][ matches[0].get(1,i+1)-1 ].get(1,1) ;
		}
#else
		for(int i=nbrFeatures[0];i--;)	
		{
			//we want to assure that the pair is recognize on both scans :
			if(i+1 == matches[1].get(1, matches[0].get(1,i+1)) )
			{
				sum_denom += featuresCartesianPoseCentered[0][i].get(1,1)*featuresCartesianPoseCentered[1][ matches[0].get(1,i+1)-1 ].get(1,1) + featuresCartesianPoseCentered[0][i].get(2,1)*featuresCartesianPoseCentered[1][ matches[0].get(1,i+1)-1 ].get(2,1) ;
			}
		}
		
		for(int i=nbrFeatures[0];i--;)
		{
			//we want to assure that the pair is recognize on both scans : idem
			if(i+1 == matches[1].get(1, matches[0].get(1,i+1)) )
			{
				sum_num += featuresCartesianPoseCentered[0][i].get(1,1)*featuresCartesianPoseCentered[1][ matches[0].get(1,i+1)-1 ].get(2,1) - featuresCartesianPoseCentered[0][i].get(2,1)*featuresCartesianPoseCentered[1][ matches[0].get(1,i+1)-1 ].get(1,1) ;
			}
		}
#endif
		
		float coeff_zarbi = 1.f;
		deltaPose.theta = coeff_zarbi*atan2( sum_num, sum_denom);
		cout << sum_num << " " << sum_denom << " dTheta = " << deltaPose.theta << endl;
		//------------------------------------------------------------------------
		//TODO : use histogram with scan points if there is less than ten feature poitns :
		
		//---------------------------------------------------------
		//---------------------------------------------------------
		Mat<float> centroidNew(rot2D(deltaPose.theta)*centroids[1]);
		Mat<float> pose( centroids[0]-centroidNew);
		deltaPose.x = -pose.get(1,1);
		deltaPose.y = -pose.get(2,1);
		
		
		//----------------------------------------------------------------
		//		INCREMENTAL ESTIMATION with initial guess :
		//----------------------------------------------------------------
		//TODO : 
		
		//----------------------------------------------------------------
		//----------------------------------------------------------------
	
	
		ROS_INFO("DELTA POSE : x = %f ; y = %f ; theta = %f.", deltaPose.x, deltaPose.y, deltaPose.theta);
		
		ROS_INFO(" CENTROIDS :");
		operatorL(centroids[0],centroidNew).afficher();
		
	}
	
	Mat<float> rot2D(float angle)
	{
		Mat<float> r(2,2);
		r.set(cos(angle), 1,1);
		r.set(cos(angle), 2,2);
		r.set(-sin(angle), 1,2);
		r.set(sin(angle), 2,1);
		return r;
	}
	
};

int main(int argc, char* argv[])
{

	ros::init(argc,argv,"HAYAI_SLAM");
	
	int rate = 100;
	if(argc>1)
		rate = atoi(argv[0]);
		
	HAYAI_SLAM instanceHS(rate);
	
	return 0;
}

	
