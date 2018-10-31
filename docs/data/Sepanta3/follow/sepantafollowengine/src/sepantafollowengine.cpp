#include <SepantaFollow.h>

actionlib::SimpleActionClient<sepanta_msgs::MasterAction> * ac;

SepantaFollowEngine::SepantaFollowEngine() : 
App_exit(false),
_thread_Logic(&SepantaFollowEngine::logic_thread,this),
_thread_10hz_publisher(&SepantaFollowEngine::scan10hz_thread,this),
it(nh)
{
    init();
}

SepantaFollowEngine::~SepantaFollowEngine()
{
	kill();
}

bool SepantaFollowEngine::isidexist(int id)
{
   for ( int i = 0 ; i < list_persons.size() ; i++ )
   {
      if ( list_persons.at(i).ID == id ) 
      {
         target_person = list_persons.at(i);
         return true;
      }

       
   }
   return false;
}

double SepantaFollowEngine::Quat2Rad(double orientation[])
{
    tf::Quaternion q(orientation[0], orientation[1], orientation[2], orientation[3]);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

double SepantaFollowEngine::Quat2Rad2(tf::Quaternion q)
{
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void SepantaFollowEngine::GetPos(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    Position[0] = msg->pose.position.x;
    Position[1] = msg->pose.position.y;
   
    orientation[0] = msg->pose.orientation.x;
    orientation[1] = msg->pose.orientation.y;
    orientation[2] = msg->pose.orientation.z;
    orientation[3] = msg->pose.orientation.w;
   
    Tetha = Quat2Rad(orientation);
    //if (Tetha < 0) Tetha += 2 * M_PI;

    //cout<<"POS : "<<Position[0]<<" "<<Position[1]<<" "<<Tetha<<endl;
}

double SepantaFollowEngine::GetDistance(double x1, double y1, double x2, double y2)
{
    double x = x2-x1;
    double y = y2-y1;
    return sqrt(x*x + y*y);
}

bool SepantaFollowEngine::find_user_for_follow()
{
    int dist_min = 100;
    bool valid = false;
    for ( int i = 0 ; i < list_persons.size() ; i++ )
    {   
        double _x = list_persons.at(i).pose.position.x;
        double _y = list_persons.at(i).pose.position.y;

        if ( _x > 0.5 && _x < 3 && abs(_y) < 0.4 )
        {
            double dist = GetDistance(0,0,_x,_y);
            if ( dist < dist_min )
            {
                dist_min = dist;
                target_person = list_persons.at(i);
                valid = true;
            }
        }
    }

    return valid;
}

void SepantaFollowEngine::change_led(int r,int g,int b)
{
  sepanta_msgs::led _msg;

  if( r != 0 || g != 0 || b != 0)
  {
     _msg.enable = true;
     _msg.colorR = r;
     _msg.colorG = g;
     _msg.colorB = b;
  }
  else
  {
    _msg.enable = false;
  }

  led_pub.publish(_msg);
}

int follow_state;
int find_state;

double goal_x;
double goal_y;
int action_state = 0;

double old_goal_x;
double old_goal_y;



void SepantaFollowEngine::logic_thread()
{
    follow_state = 0;
    find_state = 0;
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    std::cout<<"logic thread started"<<endl;
 
    while(ros::ok() && !App_exit)
    {
         boost::this_thread::sleep(boost::posix_time::milliseconds(100));

         if ( follow_state == 0 )
         {
            change_led(250,0,0);
            cout<<"[state = 0] : wait for user "<<endl;
            follow_state = 1;
            find_state = 0;
         }
         else if ( follow_state == 1 )
         {
             cout<<"[state = 1] : find_state : "<<find_state<<endl;

             bool result = find_user_for_follow();
             change_led(0,0,250);

             if ( result )
             {

                find_state++;

                if ( find_state > 2)
                {
                    find_state = 0;
                    follow_state = 2;

                }
             }
             else
             {
                find_state = 0;
             }
         }
         else if ( follow_state == 2 )
         {
         	 
             bool result = isidexist(target_person.ID);
             if ( result == false )
             {
                 change_led(250,0,0);
                 cout<<"[state = 2] User Lost"<<endl;
                 sepanta_move->exe_cancel();
                 follow_state = 0;
             }
             else
             {
                  change_led(0,250,0);
                  double e_x = Position[0]+ (target_person.pose.position.x+0.27) * cos(Tetha) - (target_person.pose.position.y) * sin(Tetha);
                  double e_y = Position[1]+ (target_person.pose.position.x+0.27) * sin(Tetha) + (target_person.pose.position.y) * cos(Tetha);
                  //double g[4];
                  //g[0] = target_person.pose.orientation.x;
                  //g[1] = target_person.pose.orientation.y;
                  //g[2] = target_person.pose.orientation.z;
                  ///g[3] = target_person.pose.orientation.w;

                  //double e_yaw = Rad2Deg(Quat2Rad(g));

                  //cout<<"YAW : "<<e_yaw<<endl;

                  double r_costmap = 0.5;
                  double Y = e_y - Position[0];
                  double X = e_x - Position[1];
                  double R = sqrt(X*X + Y*Y);
                  double r = R - r_costmap;

                  double x_goal = ( X * r ) / R;
                  double y_goal = ( x_goal * Y ) / X;


                visualization_msgs::Marker points;

                points.header.frame_id =  "map";
                points.header.stamp = ros::Time::now();
                points.ns = "point";
                points.action = visualization_msgs::Marker::ADD;
                points.pose.orientation.w = 1.0;
                points.id = 0;
                points.type = visualization_msgs::Marker::POINTS;
                points.scale.x = 0.1;
                points.scale.y = 0.1;
                points.color.r = 1;
                points.color.a = 1.0;

                geometry_msgs::Point p;
                p.x = x_goal;
                p.y = y_goal;
                p.z = 0;

                points.points.push_back(p);

                marker_pub.publish(points);
                
                goal_data l;
                l.x = (int)(x_goal * 100);
                l.y = (int)(y_goal * 100);
                l.yaw = 0;
                sepanta_move->exe_slam(l);
  
             }

         }
        
    }
}

void SepantaFollowEngine::scan10hz_thread()
{
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    std::cout<<"10hz publisher thread started"<<endl;
 
    while(ros::ok() && !App_exit)
    {
         scan10hz_can_send = true;
         boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }
}

void SepantaFollowEngine::chatterCallback_laser(const sensor_msgs::LaserScan::ConstPtr &msg)
{
   if ( scan10hz_can_send )
   {
      scan10hz_can_send = false;
      scan10hz_pub.publish(msg);
   }

}

void SepantaFollowEngine::chatterCallback_persons(const sepanta_msgs::PersonArray::ConstPtr &msg)
{
     list_persons.clear();

     for  ( int i = 0 ; i < msg->people.size() ; i++ )
     {
        person p;
        p.pose = msg->people.at(i).pose;
        p.ID = msg->people.at(i).id;
        list_persons.push_back(p);
     }

    // cout<<"people detected : "<<list_persons.size()<<endl;
}

void SepantaFollowEngine::rgbImageCallback(const sensor_msgs::ImageConstPtr& input_image) 
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(input_image, sensor_msgs::image_encodings::BGR8);
  
    cv::Size s = cv_ptr->image.size();

    //cout<<s.width<<endl;
    //cout<<s.height<<endl;

    float w = 1280;
    float h = 720;
    float ratio = w / h;
    int w2 = 800;
    int h2 = w2 / ratio;

    cv::Size size(w2,h2);//the dst image size,e.g.100x100
    cv::Mat dst;//dst image
    //Mat src;//src image
    cv::resize(cv_ptr->image,dst,size);//resize image

    //cv::imshow("Objects Visualizer", dst);
    //cv::waitKey(1);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(input_image->header, "bgr8", dst).toImageMsg();

    small_image_pub.publish(msg); 
   

}

void SepantaFollowEngine::init()
{

ROS_INFO("SepantaFollowEngine Version 1.0.0 :*");

App_exit = false;
scan10hz_can_send = false;

//============================================================================================
sub_handles[0] = node_handle.subscribe("/slam_out_pose", 10, &SepantaFollowEngine::GetPos,this);
sub_handles[1] = node_handle.subscribe("/scan",10,&SepantaFollowEngine::chatterCallback_laser,this);
sub_handles[2] = node_handle.subscribe("/people_tracked",10,&SepantaFollowEngine::chatterCallback_persons,this);
sub_handles[3] = node_handle.subscribe("/kinect2/hd/image_color_rect", 1, &SepantaFollowEngine::rgbImageCallback,this);
sub_handles[4] = node_handle.subscribe("kinect2/bodyArray",10,chatterCallback_kinect2_body);
//============================================================================================
scan10hz_pub = node_handle.advertise<sensor_msgs::LaserScan>("/scan_10hz", 10);
marker_pub =  node_handle.advertise<visualization_msgs::Marker>("visualization_marker_follow_target", 10);
led_pub = node_handle.advertise<sepanta_msgs::led>("/lowerbodycore/led", 10);
small_image_pub = it.advertise("/kinect2/small/image_color_rect", 1);

sepanta_move = new smove();

ROS_INFO("Init done");

}

void SepantaFollowEngine::kill()
{
    _thread_Logic.interrupt();
    _thread_Logic.join();

    _thread_PathFwr.interrupt();
    _thread_PathFwr.join();

    _thread_Vis.interrupt();
    _thread_Vis.join();

}