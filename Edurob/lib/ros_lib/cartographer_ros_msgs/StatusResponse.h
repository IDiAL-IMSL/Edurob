#ifndef _ROS_cartographer_ros_msgs_StatusResponse_h
#define _ROS_cartographer_ros_msgs_StatusResponse_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace cartographer_ros_msgs
{

  class StatusResponse : public ros::Msg
  {
    public:
      typedef uint8_t _code_type;
      _code_type code;
      typedef const char* _message_type;
      _message_type message;

    StatusResponse():
      code(0),
      message("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->code >> (8 * 0)) & 0xFF;
      offset += sizeof(this->code);
      uint32_t length_message = strlen(this->message);
      varToArr(outbuffer + offset, length_message);
      offset += 4;
      memcpy(outbuffer + offset, this->message, length_message);
      offset += length_message;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->code =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->code);
      uint32_t length_message;
      arrToVar(length_message, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_message; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_message-1]=0;
      this->message = (char *)(inbuffer + offset-1);
      offset += length_message;
     return offset;
    }

    virtual const char * getType() override { return "cartographer_ros_msgs/StatusResponse"; };
    virtual const char * getMD5() override { return "f45eaca0a8effd52a8b18d39434a6627"; };

  };

}
#endif
