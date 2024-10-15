#ifndef _ROS_cartographer_ros_msgs_TrajectoryStates_h
#define _ROS_cartographer_ros_msgs_TrajectoryStates_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace cartographer_ros_msgs
{

  class TrajectoryStates : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t trajectory_id_length;
      typedef int32_t _trajectory_id_type;
      _trajectory_id_type st_trajectory_id;
      _trajectory_id_type * trajectory_id;
      uint32_t trajectory_state_length;
      typedef uint8_t _trajectory_state_type;
      _trajectory_state_type st_trajectory_state;
      _trajectory_state_type * trajectory_state;
      enum { ACTIVE =  0 };
      enum { FINISHED =  1 };
      enum { FROZEN =  2 };
      enum { DELETED =  3 };

    TrajectoryStates():
      header(),
      trajectory_id_length(0), st_trajectory_id(), trajectory_id(nullptr),
      trajectory_state_length(0), st_trajectory_state(), trajectory_state(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->trajectory_id_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->trajectory_id_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->trajectory_id_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->trajectory_id_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->trajectory_id_length);
      for( uint32_t i = 0; i < trajectory_id_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_trajectory_idi;
      u_trajectory_idi.real = this->trajectory_id[i];
      *(outbuffer + offset + 0) = (u_trajectory_idi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_trajectory_idi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_trajectory_idi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_trajectory_idi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->trajectory_id[i]);
      }
      *(outbuffer + offset + 0) = (this->trajectory_state_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->trajectory_state_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->trajectory_state_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->trajectory_state_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->trajectory_state_length);
      for( uint32_t i = 0; i < trajectory_state_length; i++){
      *(outbuffer + offset + 0) = (this->trajectory_state[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->trajectory_state[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t trajectory_id_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      trajectory_id_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      trajectory_id_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      trajectory_id_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->trajectory_id_length);
      if(trajectory_id_lengthT > trajectory_id_length)
        this->trajectory_id = (int32_t*)realloc(this->trajectory_id, trajectory_id_lengthT * sizeof(int32_t));
      trajectory_id_length = trajectory_id_lengthT;
      for( uint32_t i = 0; i < trajectory_id_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_trajectory_id;
      u_st_trajectory_id.base = 0;
      u_st_trajectory_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_trajectory_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_trajectory_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_trajectory_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_trajectory_id = u_st_trajectory_id.real;
      offset += sizeof(this->st_trajectory_id);
        memcpy( &(this->trajectory_id[i]), &(this->st_trajectory_id), sizeof(int32_t));
      }
      uint32_t trajectory_state_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      trajectory_state_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      trajectory_state_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      trajectory_state_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->trajectory_state_length);
      if(trajectory_state_lengthT > trajectory_state_length)
        this->trajectory_state = (uint8_t*)realloc(this->trajectory_state, trajectory_state_lengthT * sizeof(uint8_t));
      trajectory_state_length = trajectory_state_lengthT;
      for( uint32_t i = 0; i < trajectory_state_length; i++){
      this->st_trajectory_state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_trajectory_state);
        memcpy( &(this->trajectory_state[i]), &(this->st_trajectory_state), sizeof(uint8_t));
      }
     return offset;
    }

    virtual const char * getType() override { return "cartographer_ros_msgs/TrajectoryStates"; };
    virtual const char * getMD5() override { return "85efdd795e95b57a59cb785ecb152345"; };

  };

}
#endif
