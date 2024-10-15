#ifndef _ROS_cartographer_ros_msgs_StatusCode_h
#define _ROS_cartographer_ros_msgs_StatusCode_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace cartographer_ros_msgs
{

  class StatusCode : public ros::Msg
  {
    public:
      enum { OK = 0 };
      enum { CANCELLED = 1 };
      enum { UNKNOWN = 2 };
      enum { INVALID_ARGUMENT = 3 };
      enum { DEADLINE_EXCEEDED = 4 };
      enum { NOT_FOUND = 5 };
      enum { ALREADY_EXISTS = 6 };
      enum { PERMISSION_DENIED = 7 };
      enum { RESOURCE_EXHAUSTED = 8 };
      enum { FAILED_PRECONDITION = 9 };
      enum { ABORTED = 10 };
      enum { OUT_OF_RANGE = 11 };
      enum { UNIMPLEMENTED = 12 };
      enum { INTERNAL = 13 };
      enum { UNAVAILABLE = 14 };
      enum { DATA_LOSS = 15 };

    StatusCode()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return "cartographer_ros_msgs/StatusCode"; };
    virtual const char * getMD5() override { return "90c460dc6da71af1a19af6615a8dc9a4"; };

  };

}
#endif
