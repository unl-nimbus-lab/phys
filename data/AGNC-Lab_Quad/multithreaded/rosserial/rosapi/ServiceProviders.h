#ifndef _ROS_SERVICE_ServiceProviders_h
#define _ROS_SERVICE_ServiceProviders_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rosapi
{

static const char SERVICEPROVIDERS[] = "rosapi/ServiceProviders";

  class ServiceProvidersRequest : public ros::Msg
  {
    public:
      const char* service;

    ServiceProvidersRequest():
      service("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_service = strlen(this->service);
      memcpy(outbuffer + offset, &length_service, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->service, length_service);
      offset += length_service;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_service;
      memcpy(&length_service, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_service; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_service-1]=0;
      this->service = (char *)(inbuffer + offset-1);
      offset += length_service;
     return offset;
    }

    const char * getType(){ return SERVICEPROVIDERS; };
    const char * getMD5(){ return "1cbcfa13b08f6d36710b9af8741e6112"; };

  };

  class ServiceProvidersResponse : public ros::Msg
  {
    public:
      uint32_t providers_length;
      char* st_providers;
      char* * providers;

    ServiceProvidersResponse():
      providers_length(0), providers(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->providers_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->providers_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->providers_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->providers_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->providers_length);
      for( uint32_t i = 0; i < providers_length; i++){
      uint32_t length_providersi = strlen(this->providers[i]);
      memcpy(outbuffer + offset, &length_providersi, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->providers[i], length_providersi);
      offset += length_providersi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t providers_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      providers_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      providers_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      providers_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->providers_length);
      if(providers_lengthT > providers_length)
        this->providers = (char**)realloc(this->providers, providers_lengthT * sizeof(char*));
      providers_length = providers_lengthT;
      for( uint32_t i = 0; i < providers_length; i++){
      uint32_t length_st_providers;
      memcpy(&length_st_providers, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_providers; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_providers-1]=0;
      this->st_providers = (char *)(inbuffer + offset-1);
      offset += length_st_providers;
        memcpy( &(this->providers[i]), &(this->st_providers), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return SERVICEPROVIDERS; };
    const char * getMD5(){ return "945f6849f44f061c178ab393b12c1358"; };

  };

  class ServiceProviders {
    public:
    typedef ServiceProvidersRequest Request;
    typedef ServiceProvidersResponse Response;
  };

}
#endif
