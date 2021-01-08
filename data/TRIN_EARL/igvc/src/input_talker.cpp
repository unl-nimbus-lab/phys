#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <termios.h>

using namespace ros;


char getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  char c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}



int main(int argc, char **argv)
{

  init(argc, argv, "talker");
  NodeHandle n;
  int c;
  Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {

  
// %Tag(FILL_MESSAGE)%
    std_msgs::String msg;

    std::stringstream ss;
    //ss << "hello world " << count;
   


ss<<getch();


msg.data = ss.str();
// %EndTag(FILL_MESSAGE)%

// %Tag(ROSCONSOLE)%
    ROS_INFO("%s", msg.data.c_str());
// %EndTag(ROSCONSOLE)%

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
// %Tag(PUBLISH)%
    chatter_pub.publish(msg);
// %EndTag(PUBLISH)%

// %Tag(SPINONCE)%
    spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
    ++count;
  }


  return 0;
}
// %EndTag(FULLTEXT)%

