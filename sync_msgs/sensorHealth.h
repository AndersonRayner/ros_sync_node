#ifndef _ROS_sync_msgs_sensorHealth_h
#define _ROS_sync_msgs_sensorHealth_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace sync_msgs
{

  class sensorHealth : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef uint8_t _status_type;
      _status_type status;
      typedef float _rate_type;
      _rate_type rate;
      enum { UNKNOWN =  0 };
      enum { HEALTHY =  1 };
      enum { UNHEALTHY =  2 };
      enum { MISSING =  3 };

    sensorHealth():
      name(""),
      status(0),
      rate(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      *(outbuffer + offset + 0) = (this->status >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status);
      union {
        float real;
        uint32_t base;
      } u_rate;
      u_rate.real = this->rate;
      *(outbuffer + offset + 0) = (u_rate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rate);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      this->status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->status);
      union {
        float real;
        uint32_t base;
      } u_rate;
      u_rate.base = 0;
      u_rate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rate = u_rate.real;
      offset += sizeof(this->rate);
     return offset;
    }

    virtual const char * getType() override { return "sync_msgs/sensorHealth"; };
    virtual const char * getMD5() override { return "64368f8b9e8820f939663e2bdd64ec56"; };

  };

}
#endif
