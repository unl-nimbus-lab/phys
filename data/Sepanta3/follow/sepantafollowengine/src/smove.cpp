#include <smove.h>

// #define VIRTUALMODE

smove::smove() : 
App_exit(false),
_thread_PathFwr(&smove::PathFwr,this),
_thread_Logic(&smove::logic_thread,this),
_thread_Vis(&smove::vis_thread,this)
{
    init();
}

smove::~smove()
{
	kill();
}

double smove::Quat2Rad(double orientation[])
{
    tf::Quaternion q(orientation[0], orientation[1], orientation[2], orientation[3]);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void smove::setsystemstate(int value,bool forced = false)
{
   if ( getstatemutex() || forced)
   		system_state = value;
}

void smove::setlogicstate(int value,bool forced = false)
{
	if ( getstatemutex() || forced)
   		logic_state = value;
}

int smove::getsystemstate()
{
   return system_state;
}

int smove::getlogicstate()
{
  return logic_state;
}

bool smove::getstatemutex()
{
   return statemutex;
}

void smove::setstatemutex(bool value)
{
    statemutex = value;
}

void smove::say_message(string data)
{
    if ( say_enable == false ) return;
    isttsready = false;
    sepanta_msgs::command _msg;
   _msg.request.command = data;
    say_service.call(_msg);
    sayMessageId = _msg.response.result;
    while(!isttsready)
    {
        cout<<"wait for tts id : "<<sayMessageId<<endl;
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    }
}

void smove::send_omni(double x,double y ,double w)
{
        //cout<<"publish : "<<x<<" "<<y<<" "<<w<<endl;

        geometry_msgs::Twist myTwist;

       
            myTwist.linear.x = x;
            myTwist.linear.y = -y;
            myTwist.angular.z = -w;
        
       
        mycmd_vel_pub.publish(myTwist); 
}

void smove::force_stop()
{
    //cout<<"force stop"<<endl;
    send_omni(0,0,0);
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
}

double smove::GetDistance(double x1, double y1, double x2, double y2)
{
    double x = x2-x1;
    double y = y2-y1;
    return sqrt(x*x + y*y);
}

int smove::GetCurrentStep()
{
    for(int i=0;i<globalPathSize-1;i++)
    {
        if(GetDistance(position[0],position[1],globalPath.poses[i].pose.position.x, globalPath.poses[i].pose.position.y) < GetDistance(position[0],position[1],globalPath.poses[i+1].pose.position.x, globalPath.poses[i+1].pose.position.y))
            return i;
    }
    return globalPathSize-1;
}

void smove::sepantamapengine_savemap()
{
   std_srvs::Empty _s;
   client_map_save.call(_s);
}

void smove::sepantamapengine_loadmap()
{
   std_srvs::Empty _s;
   client_map_load.call(_s);
}

void smove::clean_costmaps()
{
   std_srvs::Empty _s;
   client_resetcostmap.call(_s);
}

//cm cm degree
void smove::update_hector_origin(float x,float y,float yaw)
{
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    pub_slam_origin.publish(msg);
}

void smove::reset_hector_slam()
{
	std_msgs::String _msg;
	_msg.data = "reset";
	pub_slam_reset.publish(_msg);
}

nav_msgs::Path smove::call_make_plan()
{
    nav_msgs::GetPlan srv;

    srv.request.start.header.frame_id = "map";
    srv.request.start.pose.position.x = position[0];
    srv.request.start.pose.position.y = position[1];
    srv.request.start.pose.orientation = tf::createQuaternionMsgFromYaw(tetha);

    srv.request.goal.header.frame_id = "map";
    srv.request.goal.pose.position.x = goalPos[0];
    srv.request.goal.pose.position.y = goalPos[1];
    srv.request.goal.pose.orientation = tf::createQuaternionMsgFromYaw(goalTetha);

    srv.request.tolerance = 0.1;
    client_makeplan.call(srv);

    return srv.response.plan;
}

void smove::logic_thread()
{
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    std::cout<<"logic thread started"<<endl;
    nav_msgs::Path result;

    while(ros::ok() && !App_exit)
    {
         boost::this_thread::sleep(boost::posix_time::milliseconds(20));
       
         if ( user_tracked )
         {
                  //================================================
                  error_update_major();
                  //cout<<distacne_to_goal<<endl;

                  if ( distacne_to_goal < 0.8 )
                  {
                     ReduceLimits();
                     setsystemstate(0);
                     setlogicstate(1);
                  }
                  else 
                  {
                       
                       if ( getlogicstate() == 0 || getlogicstate() == 1)
                       {
                         ResetLimits();
                         setlogicstate(2);
                       }
 
                  }
         }
         else
         {
            setlogicstate(0);
         }
        //======================================================================
        if ( getlogicstate() == 0)
        {
            force_stop();
        }
        else
        if ( getlogicstate() == 1 )
        {
            //Look at to user
             cout<<"L 1: " <<Mtetha<<" "<<goal_desire_errorTetha<<endl;

            if(fabs(Mtetha)<=goal_desire_errorTetha)
             {
                 force_stop();
             }
             else
             {
                 controller_update2(0,false,true);
             }
        }
          else if ( getlogicstate() == 2 )
        {
             result = call_make_plan();
            //check the plan
            cout<<"Check the plan from global planner"<<endl;
           
            if( result.poses.size() == 0 )
            {
                    cout<<coutcolor_red<<"Error in PATH ! "<<coutcolor0<<endl;

                    setlogicstate(3);
                    setsystemstate(-1); //wait
                    force_stop();
                    if(!IsRecoveryState)
                        say_message("Error in path generation");
            }
            else
            {
                globalPath = result;
                globalPathSize = globalPath.poses.size();
                cout<<coutcolor_green<<"get a new PATH from GPLANNER Points : "<< globalPathSize <<coutcolor0<<endl;
                setsystemstate(1);
                setlogicstate(4);
                force_stop();   

            }
        }
        else if ( getlogicstate() == 4 )
        {
            IsRecoveryState = false;
            IsHectorReset = false;
            cout<<coutcolor_magenta<<" getlogicstate() == 2 " <<coutcolor0<<endl;

           if ( IsGoalReached == true)
           {
               cout<<coutcolor_red<<" Goal reached " <<coutcolor0<<endl;
               setlogicstate(0);
               setsystemstate(0);
               continue;
           }
            
            result = call_make_plan();
            if( result.poses.size() != 0 )
             {

            
                int currentStep = GetCurrentStep();
                for(int i=0;i<result.poses.size()-1 && i+currentStep<globalPathSize-1;i++)
                {
                    if(GetDistance(result.poses[i].pose.position.x, result.poses[i].pose.position.y,globalPath.poses[i+currentStep].pose.position.x, globalPath.poses[i+currentStep].pose.position.y)>0.2)
                    {
                        globalPath = result;
                        globalPathSize = result.poses.size();
                        setsystemstate(1);
                        cout<<coutcolor_red<<"PATH changed : "<< globalPathSize <<coutcolor0<<endl;
                        break;
                    }
                }

             }
             else
             {
                    cout<<coutcolor_red<<"Error in PATH ! "<<coutcolor0<<endl;
                    setlogicstate(3);
             }
        }
        else if ( getlogicstate() == 3 )
        {
        	say_message("Let me think");
            cout<<coutcolor_red<<" Recovery state " <<coutcolor0<<endl;
            //path error handler 
            //revocery state       
            if(IsRecoveryState)
            {
            	if(IsHectorReset)
            	{
            		IsRecoveryState = false;   
                    IsHectorReset = false;       
	            	say_message("Goal is unreachable");
	            	setsystemstate(0);
	            	setlogicstate(0);
            	}
            	else
            	{
            		IsHectorReset = true;
	            	say_message("reseting hector");
            		reset_hector_slam();
            		update_hector_origin(position[0],position[1],tetha);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
                    clean_costmaps();   
                    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
            		setlogicstate(0);
	            }

            }
            else
            {
            	IsRecoveryState = true;
            	clean_costmaps();
                boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
            	setlogicstate(0);
         	}
             
        }
     
    }
}

void smove::exe_slam(goal_data g)
{
    goalPos[0] = (float)(g.x) / 100; //cm => m
    goalPos[1] = (float)(g.y) / 100; //cm => m
    //goalTetha = Deg2Rad(g.yaw); //rad => deg

    user_tracked = true;
}

void smove::exe_cancel()
{
     user_tracked = false;

	 setstatemutex(false);
	 setlogicstate(0,true);
     setsystemstate(0,true);
     cout<<"Cancel requested"<<endl;
     setlastnavigationresult("CANCEL REQUEST");
     force_stop();
     setstatemutex(true);


}

int smove::sign(double data)
{
    if(data > 0) return 1;
    else if(data < 0) return -1;
    else return 0;
}

int smove::roundData(double data)
{
    if(data>=0)
        return ceil(data);
    else
        return floor(data);
}

double smove::GetToPointsAngle(double x1, double y1, double x2, double y2)
{
    return atan2(y2-y1,x2-x1);
}

void smove::ResetLimits()
{
    desireErrorX = normal_desire_errorX;
    desireErrorY = normal_desire_errorY;
    desireErrorTetha = normal_desire_errorTetha;

    LKpX = normal_kp_linearX;
    LKpY = normal_kp_linearY;
    WKp = norma_kp_angular;

    LKiX = normal_ki_linearX;
    LKiY = normal_ki_linearY;
    WKi = normal_ki_angular;

    maxLinSpeedX = normal_max_linear_speedX;
    maxLinSpeedY = normal_max_linear_speedY;
    maxTethaSpeed = normal_max_angular_speed;
}

void smove::ReduceLimits()
{
    desireErrorX = goal_desire_errorX;
    desireErrorY = goal_desire_errorY;
    desireErrorTetha = goal_desire_errorTetha;

    maxLinSpeedX = goal_max_linear_speedX;
    maxLinSpeedY = goal_max_linear_speedY;
    maxTethaSpeed = goal_max_angular_speed;

    LKpX = goal_kp_linearX;
    LKpY = goal_kp_linearY;
    WKp = goal_kp_angular;

    LKiX = goal_ki_linearX;
    LKiY = goal_ki_linearY;
    WKi = goal_ki_angular;
}

//0 => wait for goal
//2 => turn to target
//4 => go on path
//6 => turn to goal
//8 => reached

void smove::setrobotmove(bool value)
{
	isrobotmove = value;
}

bool smove::getrobotmove()
{
	return isrobotmove;
}

void smove::setlastnavigationresult(string value)
{
	last_navigation_result = value;
}

string smove::getlastnavigationresult()
{
	return last_navigation_result;
}


int smove::calc_next_point()
{
            bool isgoalnext = false;
            if ( step == globalPathSize-1)
            {
                on_the_goal = true;
               
            }
                
            if(step+step_size >= globalPathSize)
            {
                step = globalPathSize - 1;
                //we are very near to goal so next is the goalstep = globalPathSize-1;
                tempGoalPos[0] = goalPos[0];
                tempGoalPos[1] = goalPos[1];
                tempGoalTetha = goalTetha;
                if (tempGoalTetha < 0) tempGoalTetha += 2*M_PI;

                isgoalnext = true;
                //on_the_goal = true;

                cout<<coutcolor_magenta<<"goal calc"<<coutcolor0<<endl;

               
            }
            else
            {
                //select the point in next 20 step and calc all errors
                step +=step_size;

                tempGoalPos[0] = globalPath.poses[step].pose.position.x;
                tempGoalPos[1] = globalPath.poses[step].pose.position.y;
                tempGoalTetha = GetToPointsAngle(position[0], position[1], globalPath.poses[step].pose.position.x, globalPath.poses[step].pose.position.y);
                if (tempGoalTetha < 0) tempGoalTetha += 2*M_PI;


                 cout<<coutcolor_magenta<<"step calc"<<coutcolor0<<endl;
                 
            }

           

            return isgoalnext;
}

void smove::errors_update()
{
            //calc errorX , errorY , errorTetha 
            errorX = tempGoalPos[0]-position[0];
            errorY = tempGoalPos[1]-position[1];
            errorTetha = tempGoalTetha-tetha;

            if (errorTetha >= M_PI) errorTetha =  errorTetha - 2*M_PI;
            if (errorTetha < -M_PI) errorTetha =  errorTetha + 2*M_PI;
            //?
            if (errorTetha > 0.833*M_PI) errorTetha = 0.833*M_PI;
            if (errorTetha < -0.833*M_PI) errorTetha = -0.833*M_PI;

            errorX_R = cos(tetha)*errorX+sin(tetha)*errorY;
            errorY_R = -sin(tetha)*errorX+cos(tetha)*errorY;

            float x1 = position[0];
            float y1 = position[1];
            float x2 = goalPos[0];
            float y2 = goalPos[1];
            float d_1 = (x2 - x1); 
            float d_2 = (y2 - y1);
            distacne_to_goal = d_1 * d_1 + d_2 * d_2;
            distacne_to_goal = sqrt(distacne_to_goal);
}

void smove::error_update_major()
{
       MerrorX = goalPos[0] - position[0];
       MerrorY = goalPos[1] - position[1];
       goalTetha = GetToPointsAngle(position[0], position[1], goalPos[0], goalPos[1]);
     
       if (goalTetha < 0) goalTetha += 2*M_PI;


       Mtetha = goalTetha - tetha;

       if (Mtetha  >= M_PI) Mtetha  =  Mtetha  - 2*M_PI;
        if (Mtetha  < -M_PI) Mtetha  = Mtetha  + 2*M_PI;
        //?
        if (Mtetha  > 0.833*M_PI) Mtetha  = 0.833*M_PI;
        if (Mtetha  < -0.833*M_PI) Mtetha  = -0.833*M_PI;   

        MerrorX_R = cos(Mtetha)*MerrorX+sin(Mtetha)*MerrorY;
        MerrorY_R = -sin(Mtetha)*MerrorX+cos(Mtetha)*MerrorY;


      //  cout<< Mtetha << endl;
}

void smove::publish_info()
{
        info_counter++;
        if ( info_counter>50)
        {
            info_counter= 0;

            cout << "Speed: " << xSpeed << " - " << ySpeed << " - " << tethaSpeed << endl;
            cout << "Step: " << step << endl;
            cout << "TError: " << errorX << " - " << errorY << " - " << errorTetha << endl;
            cout << "Goal: " << fabs(goalPos[0]-position[0]) << " - " << fabs(goalPos[1]-position[1]) << " - " << fabs(goalTetha-tetha)<< endl; 
        }
}

void smove::controller_update(int x,bool y,bool theta)
{
    if ( x == 1)
    xSpeed = (fabs(errorX_R*LKpX)<=maxLinSpeedX)?(errorX_R*LKpX):sign(errorX_R)*maxLinSpeedX;
    else if ( x == 0)
    xSpeed = 0;
    else if ( x == 2)
    xSpeed = 0.3;

    if ( y )
    ySpeed = (fabs(errorY_R*LKpY)<=maxLinSpeedY)?(errorY_R*LKpY):sign(errorY_R)*maxLinSpeedY;
    else
    ySpeed = 0;

    if ( theta )
    tethaSpeed = (fabs(errorTetha*WKp)<=maxTethaSpeed)?(errorTetha*WKp):sign(errorTetha)*maxTethaSpeed;
    else
    tethaSpeed = 0;

    send_omni(xSpeed,ySpeed,tethaSpeed); 
}

void smove::controller_update2(int x,bool y,bool theta)
{
    if ( x == 1)
    xSpeed = (fabs(MerrorX_R*LKpX)<=maxLinSpeedX)?(MerrorX_R*LKpX):sign(MerrorX_R)*maxLinSpeedX;
    else if ( x == 0)
    xSpeed = 0;
    else if ( x == 2)
    xSpeed = 0.3;

    if ( y )
    ySpeed = (fabs(MerrorY_R*LKpY)<=maxLinSpeedY)?(MerrorY_R*LKpY):sign(MerrorY_R)*maxLinSpeedY;
    else
    ySpeed = 0;

    if ( theta )
    tethaSpeed = (fabs(Mtetha*WKp)<=maxTethaSpeed)?(Mtetha*WKp):sign(Mtetha)*maxTethaSpeed;
    else
    tethaSpeed = 0;

    //cout<<xSpeed<<" "<<ySpeed<<" "<<tethaSpeed<<endl;
    send_omni(xSpeed,ySpeed,tethaSpeed); 
}

void smove::PathFwr()
{
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    force_stop();
   
    #ifndef VIRTUALMODE
    say_message("Sepanta Move Base Started");
    #else
    say_message("Sepanta Move Base Started in virtual mode");
    #endif

    while (ros::ok() && !App_exit)
    {
        errors_update();

        if ( getsystemstate() == -1) //wait state
        {
           boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        }
        if ( getsystemstate() == 0)
        {
            if ( wait_flag == false)
            {
               wait_flag = true;
               cout<< coutcolor_green <<"Wait for goal ! ... "<< coutcolor0 <<endl;
            }
          
            boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        }
        else
        if ( getsystemstate() == 1)
        {
           force_stop();
           IsGoalReached = false;
           on_the_goal = false;
           ResetLimits();
           step = 0;
           wait_flag = false;
           cout<<"State = 1 -turn to target-"<<endl;

            bool resutl = calc_next_point();
            if ( resutl)
            {
                //next is the goal !
                cout<<"Next is goal =>3"<<endl;
              
                setsystemstate(3);
            }
            else
            {
                cout<<"Next is step =>2"<<endl;
                setsystemstate(2);
            }

        }
        else
        if ( getsystemstate() == 2)
        {
            //turn to goal <loop>
            
            if(fabs(errorTetha)<=desireErrorTetha)
            {
                
                cout<<"DONE ! "<<tetha<<" "<<tempGoalTetha<<" "<<errorTetha<<endl;
                setsystemstate(3);
                force_stop();
                
            }
            else
            {
                controller_update(0,false,true);
            }

        }
        else
        if ( getsystemstate() == 3)
        {
           cout<<"State = 3 -go on path- Step = "<<step<<endl;
           setsystemstate(4);
        }
        else
        if ( getsystemstate() == 4)
        {
           
            if(fabs(errorX_R)<=desireErrorX && fabs(errorY_R)<=desireErrorY && fabs(errorTetha)<=desireErrorTetha)
            {
                bool resutl = calc_next_point();
                if ( resutl )
                {
                	//next is the goal
                	ReduceLimits();

                    if ( on_the_goal )
                    {
                       setsystemstate(8);
                    }
                    else
                    {
                       setsystemstate(3);
                    }
                    
                }
                else
                {
                    cout<<"Temp point reached"<<endl;
                    setsystemstate(3);
                }
            }
            else
            {
            	if ( step < 40 || (globalPathSize - step) < 40 )
                controller_update(1,true,true); //p (X,Y,T)
                else 
                {
                	if ( fabs(errorTetha)<=desireErrorTetha )
                	{
                		controller_update(2,true,true); //fixed (F,Y,T)
                	}
                	else
                	{
                		//if we are going away ! 
                		controller_update(1,true,true); //p  fixed (X,Y,T)
                	}
                }
            }
        }
        else
        if ( getsystemstate() == 8)
        {
            cout<<"Finished !"<<endl;
            IsGoalReached = true;
            on_the_goal = false;
            wait_flag = false;
            force_stop();
            setsystemstate(0);
        }

        boost::this_thread::sleep(boost::posix_time::milliseconds(5));
    }
}

bool smove::calc_error(double x1,double y1,double t1,double x2,double y2,double t2,double delta_t)
{
   bool valid = true;
   double v1 = fabs(x2-x1) / delta_t;
   double v2 = fabs(y2-y1) / delta_t;

   double dt = t2-t1;
   if (dt >= M_PI) dt =  dt - 2*M_PI;
   if (dt < -M_PI) dt =  dt + 2*M_PI;

   double v3 = fabs(dt) / delta_t;

   if ( v1 > maxLinSpeedX + 0.1 ) valid = false;
   else if ( v2 > maxLinSpeedY + 0.1 ) valid = false;
   else if ( v3 > maxTethaSpeed + 0.1 ) valid = false;

   // cout<<v1<<" "<<v2<<" "<<v3<<" "<<delta_t<<endl;

   return valid;
}



void smove::GetPos(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    position[0] = msg->pose.position.x;
    position[1] = msg->pose.position.y;
    orientation[0] = msg->pose.orientation.x;
    orientation[1] = msg->pose.orientation.y;
    orientation[2] = msg->pose.orientation.z;
    orientation[3] = msg->pose.orientation.w;
    tetha = Quat2Rad(orientation);
    
    if ( tetha < 0)  tetha += 2*M_PI;


    //cout<<"position : "<<position[0]<<" "<<position[1]<<" "<<tetha<<endl;
}



void smove::chatterCallback_ttsfb(const std_msgs::String::ConstPtr &msg)
{
    if(!isttsready && msg->data == sayMessageId)
    {
        cout<<coutcolor_brown<<"text to speech is ready!"<<coutcolor0<<endl;
        isttsready = true;
    }
}



void smove::test_vis()
{

    visualization_msgs::Marker points , points2 , points3;

    points.header.frame_id =  "map";
    points.header.stamp = ros::Time::now();
    points.ns = "points";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.1;
    points.scale.y = 0.1;
    points.color.g = 1.0f;
    points.color.a = 1.0;

    points2.header.frame_id =  "map";
    points2.header.stamp = ros::Time::now();
    points2.action = visualization_msgs::Marker::ADD;
    points2.id = 1;
    points2.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    points2.scale.x = 1;
    points2.scale.y = 1;
    points2.scale.z = 0.4;
    points2.color.b = 0.5;
    points2.color.r = 1;
    points2.color.a = 1;

    points3.header.frame_id =  "map";
    points3.header.stamp = ros::Time::now();
    points3.action = visualization_msgs::Marker::ADD;
    points3.id = 2;
    points3.type = visualization_msgs::Marker::ARROW;
    points3.scale.x = 0.5;
    points3.scale.y = 0.05;
    points3.scale.z = 0.05;
    points3.color.b = 0.5;
    points3.color.r = 1;
    points3.color.a = 1;


     // Create the vertices for the points and lines
    for (int i = 0; i < globalPath.poses.size(); i += step_size)
    {
      
      geometry_msgs::Point p;
      p.x = globalPath.poses[i].pose.position.x;
      p.y = globalPath.poses[i].pose.position.y;
      p.z = 0;

      points.points.push_back(p);
      
    }

    for ( int i = 0 ; i < goal_list.size() ; i++)
    {
     
     points2.pose.position.x =  (float)goal_list[i].x / 100;
     points2.pose.position.y =  (float)goal_list[i].y / 100;
     points2.pose.position.z = 0;
     points2.text = goal_list[i].id;
     points2.ns = "text " + goal_list[i].id;
     marker_pub2.publish(points2);


     points3.pose.position.x =  (float)goal_list[i].x / 100;
     points3.pose.position.y =  (float)goal_list[i].y / 100;
     points3.pose.position.z = 0; 
     points3.pose.orientation = tf::createQuaternionMsgFromYaw(Deg2Rad(goal_list[i].yaw));
     points3.ns = "arrow " + goal_list[i].id;

     marker_pub3.publish(points3);
     
    }


    marker_pub.publish(points);
}

void smove::vis_thread()
{
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    while (ros::ok() && App_exit == false)
    {
    	test_vis();
        boost::this_thread::sleep(boost::posix_time::milliseconds(50));
    }
}

void smove::init()
{

ROS_INFO("smove Version 2.2 :*");

coutcolor0 = "\033[0;0m";
coutcolor_red = "\033[0;31m";
coutcolor_green = "\033[0;32m";
coutcolor_blue = "\033[0;34m";
coutcolor_magenta = "\033[0;35m";
coutcolor_brown = "\033[0;33m";
say_enable = false;
App_exit = false;
IsCmValid = false;
IsGoalReached = false;
IsRecoveryState = false;
IsHectorReset = false;
isttsready = true;
statemutex = true;
maxLinSpeedX = normal_max_linear_speedX;
maxLinSpeedY = normal_max_linear_speedY;
maxTethaSpeed = normal_max_angular_speed;
globalPathSize = 0;
temp_path_size = 0;
xSpeed=0;
ySpeed=0;
tethaSpeed=0;
desireErrorX = normal_desire_errorX;
desireErrorY = normal_desire_errorY;
desireErrorTetha = normal_desire_errorTetha;
errorX = 0;
errorY = 0;
errorTetha = 0;
errorX_R = 0;
errorY_R = 0;
LKpX = normal_kp_linearX;
LKpY = normal_kp_linearY;
WKp = norma_kp_angular;
LKiX = normal_ki_linearX;
LKiY = normal_ki_linearY;
WKi = normal_ki_angular;
step = 0;
position[2] = {0};
hectorPosition[2] = {0};
tempPosition[2] = {0};
orientation[4] = {0};
amclPosition[2] = {0};
amclOrientation[4] = {0};
tetha = 0;
hectorTetha=0;
amclTetha=0;
tempTetha = 0;
tempGoalPos[2] = {0};
tempGoalTetha = 0;
goalPos[2] = {0};
goalOri[4] = {0};
goalTetha = 0;
distacne_to_goal = 0;
maxErrorX = 0;
maxErrorY = 0;
maxErrorTetha = 0;
info_counter = 0;
system_state = 0;
logic_state = 1;
on_the_goal = false;
step_size  = 40;
wait_flag = false;
idle_flag = false;
isrobotmove = false;
last_navigation_result = "";
f = 0.0;
user_tracked = false;

    //============================================================================================
    sub_handles[0] = node_handles[0].subscribe("/slam_out_pose", 10, &smove::GetPos,this);
    //============================================================================================
    mycmd_vel_pub = node_handles[3].advertise<geometry_msgs::Twist>("sepantamovebase/cmd_vel", 10);
    pub_slam_origin = node_handles[4].advertise<geometry_msgs::PoseWithCovarianceStamped>("/slam_origin", 1);
    pub_slam_reset = node_handles[5].advertise<std_msgs::String>("syscommand", 1);
    //============================================================================================
    pub_tts = node_handles[6].advertise<std_msgs::String>("/texttospeech/message", 10);
    //============================================================================================
    pub_current_goal = node_handles[7].advertise<geometry_msgs::PoseStamped>("current_goal", 0 );
    pub_alarm = node_handles[7].advertise<std_msgs::Int32>("lowerbodycore/alarm",10);
    pub_move = node_handles[7].advertise<std_msgs::Bool>("lowerbodycore/isrobotmove", 10);
    //============================================================================================
    marker_pub =  node_handles[7].advertise<visualization_msgs::Marker>("visualization_marker_steps", 10);
    marker_pub2 =  node_handles[7].advertise<visualization_msgs::Marker>("visualization_marker_goals", 10);
    marker_pub3 =  node_handles[7].advertise<visualization_msgs::Marker>("visualization_marker_goals_arrow", 10);
    //============================================================================================
    client_makeplan = node_handles[9].serviceClient<nav_msgs::GetPlanRequest>("move_base/make_plan");
 	client_resetcostmap = node_handles[10].serviceClient<std_srvs::EmptyRequest>("move_base/clear_costmaps");
 	client_map_save = node_handles[11].serviceClient<std_srvs::EmptyRequest>("sepantamapengenine/save");
    client_map_load = node_handles[12].serviceClient<std_srvs::EmptyRequest>("sepantamapengenine/load");
    //============================================================================================
    sub_handles[4] = node_handles[3].subscribe("/texttospeech/queue", 10, &smove::chatterCallback_ttsfb,this);
    say_service = node_handles[11].serviceClient<sepanta_msgs::command>("texttospeech/say");
    //============================================================================================
   
    ROS_INFO("Init done");
}

void smove::kill()
{
	_thread_PathFwr.interrupt();
    _thread_PathFwr.join();

    _thread_Logic.interrupt();
    _thread_Logic.join();

    _thread_Vis.interrupt();
    _thread_Vis.join();
}