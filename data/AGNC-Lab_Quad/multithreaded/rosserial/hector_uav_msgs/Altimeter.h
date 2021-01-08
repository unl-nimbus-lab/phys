#ifndef _ROS_hector_uav_msgs_Altimeter_h
#define _ROS_hector_uav_msgs_Altimeter_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace hector_uav_msgs
{

  class Altimeter : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float altitude;
      float pressure;
      float qnh;

    Altimeter():
      header(),
      altitude(0),
      pressure(0),
      qnh(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_altitude;
      u_altitude.real = this->altitude;
      *(outbuffer + offset + 0) = (u_altitude.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_altitude.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_altitude.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_altitude.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->altitude);
      union {
        float real;
        uint32_t base;
      } u_pressure;
      u_pressure.real = this->pressure;
      *(outbuffer + offset + 0) = (u_pressure.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pressure.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pressure.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pressure.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pressure);
      union {
        float real;
        uint32_t base;
      } u_qnh;
      u_qnh.real = this->qnh;
      *(outbuffer + offset + 0) = (u_qnh.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_qnh.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_qnh.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_qnh.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->qnh);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_altitude;
      u_altitude.base = 0;
      u_altitude.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_altitude.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_altitude.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_altitude.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->altitude = u_altitude.real;
      offset += sizeof(this->altitude);
      union {
        float real;
        uint32_t base;
      } u_pressure;
      u_pressure.base = 0;
      u_pressure.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pressure.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pressure.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pressure.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pressure = u_pressure.real;
      offset += sizeof(this->pressure);
      union {
        float real;
        uint32_t base;
      } u_qnh;
      u_qnh.base = 0;
      u_qnh.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_qnh.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_qnh.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_qnh.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->qnh = u_qnh.real;
      offset += sizeof(this->qnh);
     return offset;
    }

    const char * getType(){ return "hector_uav_msgs/Altimeter"; };
    const char * getMD5(){ return "c785451e2f67a76b902818138e9b53c6"; };

  };

}
#endif