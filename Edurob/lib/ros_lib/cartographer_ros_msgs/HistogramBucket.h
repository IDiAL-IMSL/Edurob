#ifndef _ROS_cartographer_ros_msgs_HistogramBucket_h
#define _ROS_cartographer_ros_msgs_HistogramBucket_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace cartographer_ros_msgs
{

  class HistogramBucket : public ros::Msg
  {
    public:
      typedef float _bucket_boundary_type;
      _bucket_boundary_type bucket_boundary;
      typedef float _count_type;
      _count_type count;

    HistogramBucket():
      bucket_boundary(0),
      count(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->bucket_boundary);
      offset += serializeAvrFloat64(outbuffer + offset, this->count);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->bucket_boundary));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->count));
     return offset;
    }

    virtual const char * getType() override { return "cartographer_ros_msgs/HistogramBucket"; };
    virtual const char * getMD5() override { return "b579df35b32758cf09f65ae223aea0ad"; };

  };

}
#endif
