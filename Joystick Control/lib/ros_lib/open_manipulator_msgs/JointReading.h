#ifndef _ROS_open_manipulator_msgs_JointReading_h
#define _ROS_open_manipulator_msgs_JointReading_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace open_manipulator_msgs
{

  class JointReading : public ros::Msg
  {
    public:
      typedef uint8_t _id_type;
      _id_type id;
      typedef float _position_type;
      _position_type position;
      typedef float _velocity_type;
      _velocity_type velocity;
      typedef float _effort_type;
      _effort_type effort;

    JointReading():
      id(0),
      position(0),
      velocity(0),
      effort(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      offset += sizeof(this->id);
      offset += serializeAvrFloat64(outbuffer + offset, this->position);
      offset += serializeAvrFloat64(outbuffer + offset, this->velocity);
      offset += serializeAvrFloat64(outbuffer + offset, this->effort);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->id =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->id);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->position));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->velocity));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->effort));
     return offset;
    }

    virtual const char * getType() override { return "open_manipulator_msgs/JointReading"; };
    virtual const char * getMD5() override { return "9f97d568912af0cb8396da157d86d126"; };

  };

}
#endif
