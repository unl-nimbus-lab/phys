#ifndef _ROS_vicon_bridge_TfDistortInfo_h
#define _ROS_vicon_bridge_TfDistortInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace vicon_bridge
{

  class TfDistortInfo : public ros::Msg
  {
    public:
      double tf_pub_rate;
      const char* tf_ref_frame;
      const char* tf_frame_in;
      const char* tf_frame_out;
      int32_t delay;
      double position_scale;
      const char* noise_type;
      double sigma_roll_pitch;
      double sigma_yaw;
      double sigma_xy;
      double sigma_z;
      double random_walk_k_xy;
      double random_walk_k_z;
      double random_walk_sigma_xy;
      double random_walk_sigma_z;
      double random_walk_tau_xy;
      double random_walk_tau_z;

    TfDistortInfo():
      tf_pub_rate(0),
      tf_ref_frame(""),
      tf_frame_in(""),
      tf_frame_out(""),
      delay(0),
      position_scale(0),
      noise_type(""),
      sigma_roll_pitch(0),
      sigma_yaw(0),
      sigma_xy(0),
      sigma_z(0),
      random_walk_k_xy(0),
      random_walk_k_z(0),
      random_walk_sigma_xy(0),
      random_walk_sigma_z(0),
      random_walk_tau_xy(0),
      random_walk_tau_z(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_tf_pub_rate;
      u_tf_pub_rate.real = this->tf_pub_rate;
      *(outbuffer + offset + 0) = (u_tf_pub_rate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tf_pub_rate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tf_pub_rate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tf_pub_rate.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_tf_pub_rate.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_tf_pub_rate.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_tf_pub_rate.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_tf_pub_rate.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->tf_pub_rate);
      uint32_t length_tf_ref_frame = strlen(this->tf_ref_frame);
      memcpy(outbuffer + offset, &length_tf_ref_frame, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->tf_ref_frame, length_tf_ref_frame);
      offset += length_tf_ref_frame;
      uint32_t length_tf_frame_in = strlen(this->tf_frame_in);
      memcpy(outbuffer + offset, &length_tf_frame_in, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->tf_frame_in, length_tf_frame_in);
      offset += length_tf_frame_in;
      uint32_t length_tf_frame_out = strlen(this->tf_frame_out);
      memcpy(outbuffer + offset, &length_tf_frame_out, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->tf_frame_out, length_tf_frame_out);
      offset += length_tf_frame_out;
      union {
        int32_t real;
        uint32_t base;
      } u_delay;
      u_delay.real = this->delay;
      *(outbuffer + offset + 0) = (u_delay.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_delay.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_delay.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_delay.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->delay);
      union {
        double real;
        uint64_t base;
      } u_position_scale;
      u_position_scale.real = this->position_scale;
      *(outbuffer + offset + 0) = (u_position_scale.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position_scale.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position_scale.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position_scale.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_position_scale.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_position_scale.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_position_scale.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_position_scale.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->position_scale);
      uint32_t length_noise_type = strlen(this->noise_type);
      memcpy(outbuffer + offset, &length_noise_type, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->noise_type, length_noise_type);
      offset += length_noise_type;
      union {
        double real;
        uint64_t base;
      } u_sigma_roll_pitch;
      u_sigma_roll_pitch.real = this->sigma_roll_pitch;
      *(outbuffer + offset + 0) = (u_sigma_roll_pitch.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sigma_roll_pitch.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sigma_roll_pitch.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sigma_roll_pitch.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_sigma_roll_pitch.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_sigma_roll_pitch.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_sigma_roll_pitch.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_sigma_roll_pitch.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->sigma_roll_pitch);
      union {
        double real;
        uint64_t base;
      } u_sigma_yaw;
      u_sigma_yaw.real = this->sigma_yaw;
      *(outbuffer + offset + 0) = (u_sigma_yaw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sigma_yaw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sigma_yaw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sigma_yaw.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_sigma_yaw.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_sigma_yaw.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_sigma_yaw.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_sigma_yaw.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->sigma_yaw);
      union {
        double real;
        uint64_t base;
      } u_sigma_xy;
      u_sigma_xy.real = this->sigma_xy;
      *(outbuffer + offset + 0) = (u_sigma_xy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sigma_xy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sigma_xy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sigma_xy.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_sigma_xy.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_sigma_xy.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_sigma_xy.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_sigma_xy.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->sigma_xy);
      union {
        double real;
        uint64_t base;
      } u_sigma_z;
      u_sigma_z.real = this->sigma_z;
      *(outbuffer + offset + 0) = (u_sigma_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sigma_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sigma_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sigma_z.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_sigma_z.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_sigma_z.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_sigma_z.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_sigma_z.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->sigma_z);
      union {
        double real;
        uint64_t base;
      } u_random_walk_k_xy;
      u_random_walk_k_xy.real = this->random_walk_k_xy;
      *(outbuffer + offset + 0) = (u_random_walk_k_xy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_random_walk_k_xy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_random_walk_k_xy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_random_walk_k_xy.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_random_walk_k_xy.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_random_walk_k_xy.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_random_walk_k_xy.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_random_walk_k_xy.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->random_walk_k_xy);
      union {
        double real;
        uint64_t base;
      } u_random_walk_k_z;
      u_random_walk_k_z.real = this->random_walk_k_z;
      *(outbuffer + offset + 0) = (u_random_walk_k_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_random_walk_k_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_random_walk_k_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_random_walk_k_z.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_random_walk_k_z.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_random_walk_k_z.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_random_walk_k_z.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_random_walk_k_z.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->random_walk_k_z);
      union {
        double real;
        uint64_t base;
      } u_random_walk_sigma_xy;
      u_random_walk_sigma_xy.real = this->random_walk_sigma_xy;
      *(outbuffer + offset + 0) = (u_random_walk_sigma_xy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_random_walk_sigma_xy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_random_walk_sigma_xy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_random_walk_sigma_xy.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_random_walk_sigma_xy.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_random_walk_sigma_xy.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_random_walk_sigma_xy.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_random_walk_sigma_xy.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->random_walk_sigma_xy);
      union {
        double real;
        uint64_t base;
      } u_random_walk_sigma_z;
      u_random_walk_sigma_z.real = this->random_walk_sigma_z;
      *(outbuffer + offset + 0) = (u_random_walk_sigma_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_random_walk_sigma_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_random_walk_sigma_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_random_walk_sigma_z.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_random_walk_sigma_z.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_random_walk_sigma_z.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_random_walk_sigma_z.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_random_walk_sigma_z.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->random_walk_sigma_z);
      union {
        double real;
        uint64_t base;
      } u_random_walk_tau_xy;
      u_random_walk_tau_xy.real = this->random_walk_tau_xy;
      *(outbuffer + offset + 0) = (u_random_walk_tau_xy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_random_walk_tau_xy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_random_walk_tau_xy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_random_walk_tau_xy.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_random_walk_tau_xy.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_random_walk_tau_xy.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_random_walk_tau_xy.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_random_walk_tau_xy.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->random_walk_tau_xy);
      union {
        double real;
        uint64_t base;
      } u_random_walk_tau_z;
      u_random_walk_tau_z.real = this->random_walk_tau_z;
      *(outbuffer + offset + 0) = (u_random_walk_tau_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_random_walk_tau_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_random_walk_tau_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_random_walk_tau_z.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_random_walk_tau_z.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_random_walk_tau_z.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_random_walk_tau_z.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_random_walk_tau_z.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->random_walk_tau_z);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_tf_pub_rate;
      u_tf_pub_rate.base = 0;
      u_tf_pub_rate.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tf_pub_rate.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tf_pub_rate.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tf_pub_rate.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_tf_pub_rate.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_tf_pub_rate.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_tf_pub_rate.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_tf_pub_rate.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->tf_pub_rate = u_tf_pub_rate.real;
      offset += sizeof(this->tf_pub_rate);
      uint32_t length_tf_ref_frame;
      memcpy(&length_tf_ref_frame, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_tf_ref_frame; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_tf_ref_frame-1]=0;
      this->tf_ref_frame = (char *)(inbuffer + offset-1);
      offset += length_tf_ref_frame;
      uint32_t length_tf_frame_in;
      memcpy(&length_tf_frame_in, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_tf_frame_in; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_tf_frame_in-1]=0;
      this->tf_frame_in = (char *)(inbuffer + offset-1);
      offset += length_tf_frame_in;
      uint32_t length_tf_frame_out;
      memcpy(&length_tf_frame_out, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_tf_frame_out; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_tf_frame_out-1]=0;
      this->tf_frame_out = (char *)(inbuffer + offset-1);
      offset += length_tf_frame_out;
      union {
        int32_t real;
        uint32_t base;
      } u_delay;
      u_delay.base = 0;
      u_delay.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_delay.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_delay.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_delay.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->delay = u_delay.real;
      offset += sizeof(this->delay);
      union {
        double real;
        uint64_t base;
      } u_position_scale;
      u_position_scale.base = 0;
      u_position_scale.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position_scale.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position_scale.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position_scale.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_position_scale.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_position_scale.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_position_scale.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_position_scale.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->position_scale = u_position_scale.real;
      offset += sizeof(this->position_scale);
      uint32_t length_noise_type;
      memcpy(&length_noise_type, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_noise_type; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_noise_type-1]=0;
      this->noise_type = (char *)(inbuffer + offset-1);
      offset += length_noise_type;
      union {
        double real;
        uint64_t base;
      } u_sigma_roll_pitch;
      u_sigma_roll_pitch.base = 0;
      u_sigma_roll_pitch.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sigma_roll_pitch.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sigma_roll_pitch.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sigma_roll_pitch.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_sigma_roll_pitch.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_sigma_roll_pitch.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_sigma_roll_pitch.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_sigma_roll_pitch.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->sigma_roll_pitch = u_sigma_roll_pitch.real;
      offset += sizeof(this->sigma_roll_pitch);
      union {
        double real;
        uint64_t base;
      } u_sigma_yaw;
      u_sigma_yaw.base = 0;
      u_sigma_yaw.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sigma_yaw.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sigma_yaw.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sigma_yaw.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_sigma_yaw.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_sigma_yaw.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_sigma_yaw.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_sigma_yaw.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->sigma_yaw = u_sigma_yaw.real;
      offset += sizeof(this->sigma_yaw);
      union {
        double real;
        uint64_t base;
      } u_sigma_xy;
      u_sigma_xy.base = 0;
      u_sigma_xy.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sigma_xy.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sigma_xy.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sigma_xy.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_sigma_xy.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_sigma_xy.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_sigma_xy.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_sigma_xy.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->sigma_xy = u_sigma_xy.real;
      offset += sizeof(this->sigma_xy);
      union {
        double real;
        uint64_t base;
      } u_sigma_z;
      u_sigma_z.base = 0;
      u_sigma_z.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sigma_z.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sigma_z.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sigma_z.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_sigma_z.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_sigma_z.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_sigma_z.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_sigma_z.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->sigma_z = u_sigma_z.real;
      offset += sizeof(this->sigma_z);
      union {
        double real;
        uint64_t base;
      } u_random_walk_k_xy;
      u_random_walk_k_xy.base = 0;
      u_random_walk_k_xy.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_random_walk_k_xy.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_random_walk_k_xy.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_random_walk_k_xy.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_random_walk_k_xy.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_random_walk_k_xy.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_random_walk_k_xy.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_random_walk_k_xy.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->random_walk_k_xy = u_random_walk_k_xy.real;
      offset += sizeof(this->random_walk_k_xy);
      union {
        double real;
        uint64_t base;
      } u_random_walk_k_z;
      u_random_walk_k_z.base = 0;
      u_random_walk_k_z.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_random_walk_k_z.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_random_walk_k_z.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_random_walk_k_z.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_random_walk_k_z.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_random_walk_k_z.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_random_walk_k_z.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_random_walk_k_z.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->random_walk_k_z = u_random_walk_k_z.real;
      offset += sizeof(this->random_walk_k_z);
      union {
        double real;
        uint64_t base;
      } u_random_walk_sigma_xy;
      u_random_walk_sigma_xy.base = 0;
      u_random_walk_sigma_xy.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_random_walk_sigma_xy.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_random_walk_sigma_xy.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_random_walk_sigma_xy.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_random_walk_sigma_xy.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_random_walk_sigma_xy.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_random_walk_sigma_xy.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_random_walk_sigma_xy.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->random_walk_sigma_xy = u_random_walk_sigma_xy.real;
      offset += sizeof(this->random_walk_sigma_xy);
      union {
        double real;
        uint64_t base;
      } u_random_walk_sigma_z;
      u_random_walk_sigma_z.base = 0;
      u_random_walk_sigma_z.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_random_walk_sigma_z.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_random_walk_sigma_z.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_random_walk_sigma_z.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_random_walk_sigma_z.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_random_walk_sigma_z.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_random_walk_sigma_z.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_random_walk_sigma_z.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->random_walk_sigma_z = u_random_walk_sigma_z.real;
      offset += sizeof(this->random_walk_sigma_z);
      union {
        double real;
        uint64_t base;
      } u_random_walk_tau_xy;
      u_random_walk_tau_xy.base = 0;
      u_random_walk_tau_xy.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_random_walk_tau_xy.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_random_walk_tau_xy.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_random_walk_tau_xy.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_random_walk_tau_xy.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_random_walk_tau_xy.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_random_walk_tau_xy.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_random_walk_tau_xy.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->random_walk_tau_xy = u_random_walk_tau_xy.real;
      offset += sizeof(this->random_walk_tau_xy);
      union {
        double real;
        uint64_t base;
      } u_random_walk_tau_z;
      u_random_walk_tau_z.base = 0;
      u_random_walk_tau_z.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_random_walk_tau_z.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_random_walk_tau_z.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_random_walk_tau_z.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_random_walk_tau_z.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_random_walk_tau_z.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_random_walk_tau_z.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_random_walk_tau_z.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->random_walk_tau_z = u_random_walk_tau_z.real;
      offset += sizeof(this->random_walk_tau_z);
     return offset;
    }

    const char * getType(){ return "vicon_bridge/TfDistortInfo"; };
    const char * getMD5(){ return "a7025291415a53264a70b836a949be8d"; };

  };

}
#endif