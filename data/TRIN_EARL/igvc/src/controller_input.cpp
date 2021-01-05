#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include "std_msgs/String.h"
#include <sstream>
#include <termios.h>

#include <ros/ros.h>

#include <sensor_msgs/Joy.h>

#include "rosserial_server/serial_session.h"

using namespace ros;


int getch()
{
	static struct termios oldt, newt;
	tcgetattr( STDIN_FILENO, &oldt);           // save old settings
	newt = oldt;
	newt.c_lflag &= ~(ICANON);                 // disable buffering      
	tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

	int c = getchar();  // read character (non-blocking)

	tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
	return c;
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "rosserial_server_serial_node");
	std::string port;
	ros::param::param<std::string>("~port", port, "/dev/ttyACM1");
	int baud;
	ros::param::param<int>("~baud", baud, 57600);


	boost::asio::io_service io_service;
	new rosserial_server::SerialSession(io_service, port, baud);
	boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));

	NodeHandle n;
	Publisher chatter_pub = n.advertise<std_msgs::String>("Direction", 1000);

	Rate loop_rate(10);

	int count = 0;
	while (ros::ok())
	{


		// %Tag(FILL_MESSAGE)%
		std_msgs::String msg;

		std::stringstream ss,ss1;
		//ss << "hello world " << count;

		char c = getch();   // call your non-blocking input function
		ss << "keyboard command is " << c << "\n";
		ss1 << c << "";

		msg.data = ss1.str();
		// %EndTag(FILL_MESSAGE)%

		// %Tag(ROSCONSOLE)%
		//ROS_INFO("%s", msg.data.c_str());
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

		chatter_pub.publish(msg);
		spinOnce();

		// %Tag(RATE_SLEEP)%
		//loop_rate.sleep();
		// %EndTag(RATE_SLEEP)%
		++count;
	}


	return 0;
}
// %EndTag(FULLTEXT)%




