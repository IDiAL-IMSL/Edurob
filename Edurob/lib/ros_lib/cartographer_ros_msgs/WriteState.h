#ifndef _ROS_SERVICE_WriteState_h
#define _ROS_SERVICE_WriteState_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "cartographer_ros_msgs/StatusResponse.h"

namespace cartographer_ros_msgs
{

static const char WRITESTATE[] = "cartographer_ros_msgs/WriteState";

  class WriteStateRequest : public ros::Msg
  {
    public:
      typedef const char* _filename_type;
      _filename_type filename;
      typedef bool _include_unfinished_submaps_type;
      _include_unfinished_submaps_type include_unfinished_submaps;

    WriteStateRequest():
      filename(""),
      include_unfinished_submaps(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_filename = strlen(this->filename);
      varToArr(outbuffer + offset, length_filename);
      offset += 4;
      memcpy(outbuffer + offset, this->filename, length_filename);
      offset += length_filename;
      union {
        bool real;
        uint8_t base;
      } u_include_unfinished_submaps;
      u_include_unfinished_submaps.real = this->include_unfinished_submaps;
      *(outbuffer + offset + 0) = (u_include_unfinished_submaps.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->include_unfinished_submaps);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_filename;
      arrToVar(length_filename, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_filename; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_filename-1]=0;
      this->filename = (char *)(inbuffer + offset-1);
      offset += length_filename;
      union {
        bool real;
        uint8_t base;
      } u_include_unfinished_submaps;
      u_include_unfinished_submaps.base = 0;
      u_include_unfinished_submaps.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->include_unfinished_submaps = u_include_unfinished_submaps.real;
      offset += sizeof(this->include_unfinished_submaps);
     return offset;
    }

    virtual const char * getType() override { return WRITESTATE; };
    virtual const char * getMD5() override { return "bfd12117d83df4fe52e78631c0c6b702"; };

  };

  class WriteStateResponse : public ros::Msg
  {
    public:
      typedef cartographer_ros_msgs::StatusResponse _status_type;
      _status_type status;

    WriteStateResponse():
      status()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->status.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->status.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return WRITESTATE; };
    virtual const char * getMD5() override { return "4e6ca4e44081fa06b258fa12804ea7cb"; };

  };

  class WriteState {
    public:
    typedef WriteStateRequest Request;
    typedef WriteStateResponse Response;
  };

}
#endif
