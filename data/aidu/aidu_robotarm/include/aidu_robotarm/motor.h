#ifndef AIDU_ROBOT_ARM__MOTOR_H
#define AIDU_ROBOT_ARM__MOTOR_H

#include <ros/ros.h>
#include <threemxl/C3mxlROS.h>
#include <CDxlGeneric.h>
#include <CDxlGroup.h>
#include <string>

namespace aidu {
  namespace mobile_robot_arm {
    class Motor {
    public:
      
      Motor(std::string name, std::string motor_port_name, std::string motor_config_name);
      std::string name;
      ~Motor();
      void setPosition(float position, float speed);///< Sets the position of the motor
      void setLinearPosition(float position, float speed);///< Sets the position of the motor
      void setTorque(double torque, double speed);
      CDxlGeneric *motor;               ///< The motor interface
      float currentVelocity;

      double initialPos;
      
      void reset();
      void update();
      void initialize(double speed, double torque);
      
      void setSpeed(float speed);
      double getPosition();
      double getLinearPosition();
      double getSpeed();
      
    protected:
      CDxlConfig *config;               ///< The motor configuration
      LxSerial* serial_port;            ///< The serial port
      
    };
  }
}

#endif
