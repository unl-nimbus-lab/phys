
// IGNORE this file. Refer to xbox_drive.py

# include <stdlib.h> 

# include <boost/asio.hpp>
# include <boost/bind.hpp>
# include <boost/thread.hpp>

# include "std_msgs/String.h"
# include "std_msgs/Int8.h"
# include <sstream>
# include <termios.h>

# include <sensor_msgs/Joy.h>

# include <ros/ros.h>

# include "rosserial_server/serial_session.h"


using namespace ros;

// constants taken from Q's labview code
float speed_scale = 0.01220703125;
float turn_scale = 0.06103515625;

// other constants
float speed_deadzone = 0.25;
float turn_deadzone = 0.25;

int speed = 0;
int turn = 0;

// signum function
int sgn(float x) {
    return (x > 0.0) ? 1 : ((x < 0.0) ? -1 : 0);
}

/**
 * Call back for topics recived from joy node
 * 
 * calculated speed is in range -55 to 55
 * calculated turn is in range -11 to 11
 *
 */
void callback(const sensor_msgs::Joy::ConstPtr& msg) {
  float y_axis = msg->axes[1];
  float x_axis_rotation = msg->axes[2];

  // calculate speed
  if (std::abs(y_axis) < speed_deadzone) {
    speed = 0;
  }
  else {
    speed = (y_axis - (speed_deadzone*sgn(y_axis))) / speed_scale;
  }

  // calculate turn
  if (std::abs(x_axis_rotation) < turn_deadzone) {
    turn = 0;
  }
  else {
    turn = (x_axis_rotation - (turn_deadzone*sgn(x_axis_rotation))) / turn_scale;
  }
}

int main(int argc, char **argv) {

    // initialize node and handles
    ros::init(argc, argv, "xbox_drive");
    NodeHandle n;
    NodeHandle out;

    // initialize publishers
    Publisher speed_pub = out.advertise<std_msgs::Int8>("motor_speed", 1);
    Publisher turn_pub = out.advertise<std_msgs::Int8>("motor_turn", 1);

    // initialize subscriber and register callback
    ros::Subscriber sub = n.subscribe("joy", 1000, callback);

    Rate loop_rate(10);
    while (ros::ok()) {
        // create messages
        std_msgs::Int8 msg_speed, msg_turn;
        msg_speed.data = speed;
        msg_turn.data = turn;

        // publish motor speed and turn values
        speed_pub.publish(msg_speed);
        turn_pub.publish(msg_turn);
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}




/*

to connect xbox controller: sudo rmmod xpad then sudo xboxdrv --silent


Data from Q
SPEED
y-Axis : left analog up and down, when all the way up it is -32768, when all the way down it is 32767
-32768 scales to 61,  32767 scales to -62
speed cale factor: 400   speed deadzone: 10240


TURN
x-axis rotation: right analog left and right, when all the way to the left -32768, when all the way to the right 32767
-32768 scales to -17, 32767 scales to 16 
turn scale factor 2000    turn deadzone 10240

SCALING
scale deadzone (value is either y-axis or x-asis rotation)

if abs(value) <= deadzone:
    return 0

else
    ((value - deadzone*signum(value)) / scale_factor
*/