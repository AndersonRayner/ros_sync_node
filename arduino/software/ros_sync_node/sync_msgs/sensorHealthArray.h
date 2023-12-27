#ifndef _ROS_sync_msgs_sensorHealthArray_h
#define _ROS_sync_msgs_sensorHealthArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "sensorHealth.h"

namespace sync_msgs
{

  class sensorHealthArray : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint8_t _system_type;
      _system_type system;
      uint32_t sensor_length;
      typedef sync_msgs::sensorHealth _sensor_type;
      _sensor_type st_sensor;
      _sensor_type * sensor;

    sensorHealthArray():
      header(),
      system(0),
      sensor_length(0), st_sensor(), sensor(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->system >> (8 * 0)) & 0xFF;
      offset += sizeof(this->system);
      *(outbuffer + offset + 0) = (this->sensor_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sensor_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sensor_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sensor_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sensor_length);
      for( uint32_t i = 0; i < sensor_length; i++){
      offset += this->sensor[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->system =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->system);
      uint32_t sensor_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      sensor_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      sensor_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      sensor_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->sensor_length);
      if(sensor_lengthT > sensor_length)
        this->sensor = (sync_msgs::sensorHealth*)realloc(this->sensor, sensor_lengthT * sizeof(sync_msgs::sensorHealth));
      sensor_length = sensor_lengthT;
      for( uint32_t i = 0; i < sensor_length; i++){
      offset += this->st_sensor.deserialize(inbuffer + offset);
        memcpy( &(this->sensor[i]), &(this->st_sensor), sizeof(sync_msgs::sensorHealth));
      }
     return offset;
    }

    virtual const char * getType() override { return "sync_msgs/sensorHealthArray"; };
    virtual const char * getMD5() override { return "0bded5ad499e7562dba5271bd74f56be"; };

  };

}
#endif
