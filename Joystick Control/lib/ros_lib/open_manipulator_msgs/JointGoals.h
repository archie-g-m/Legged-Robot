#ifndef _ROS_open_manipulator_msgs_JointGoals_h
#define _ROS_open_manipulator_msgs_JointGoals_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace open_manipulator_msgs
{

  class JointGoals : public ros::Msg
  {
    public:
      uint32_t goals_length;
      typedef float _goals_type;
      _goals_type st_goals;
      _goals_type * goals;

    JointGoals():
      goals_length(0), st_goals(), goals(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->goals_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->goals_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->goals_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->goals_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->goals_length);
      for( uint32_t i = 0; i < goals_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->goals[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t goals_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      goals_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      goals_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      goals_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->goals_length);
      if(goals_lengthT > goals_length)
        this->goals = (float*)realloc(this->goals, goals_lengthT * sizeof(float));
      goals_length = goals_lengthT;
      for( uint32_t i = 0; i < goals_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_goals));
        memcpy( &(this->goals[i]), &(this->st_goals), sizeof(float));
      }
     return offset;
    }

    virtual const char * getType() override { return "open_manipulator_msgs/JointGoals"; };
    virtual const char * getMD5() override { return "c1ee3e7cef8eb48c2bf81081604fc821"; };

  };

}
#endif
