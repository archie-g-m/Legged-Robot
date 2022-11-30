#ifndef _ROS_open_manipulator_msgs_JointReadings_h
#define _ROS_open_manipulator_msgs_JointReadings_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "open_manipulator_msgs/JointReading.h"

namespace open_manipulator_msgs
{

  class JointReadings : public ros::Msg
  {
    public:
      uint32_t readings_length;
      typedef open_manipulator_msgs::JointReading _readings_type;
      _readings_type st_readings;
      _readings_type * readings;

    JointReadings():
      readings_length(0), st_readings(), readings(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->readings_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->readings_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->readings_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->readings_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->readings_length);
      for( uint32_t i = 0; i < readings_length; i++){
      offset += this->readings[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t readings_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      readings_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      readings_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      readings_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->readings_length);
      if(readings_lengthT > readings_length)
        this->readings = (open_manipulator_msgs::JointReading*)realloc(this->readings, readings_lengthT * sizeof(open_manipulator_msgs::JointReading));
      readings_length = readings_lengthT;
      for( uint32_t i = 0; i < readings_length; i++){
      offset += this->st_readings.deserialize(inbuffer + offset);
        memcpy( &(this->readings[i]), &(this->st_readings), sizeof(open_manipulator_msgs::JointReading));
      }
     return offset;
    }

    virtual const char * getType() override { return "open_manipulator_msgs/JointReadings"; };
    virtual const char * getMD5() override { return "a8aca6bf9a85ce478544b3704930734f"; };

  };

}
#endif
