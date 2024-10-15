#ifndef _ROS_SERVICE_ReadMetrics_h
#define _ROS_SERVICE_ReadMetrics_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "cartographer_ros_msgs/StatusResponse.h"
#include "ros/time.h"
#include "cartographer_ros_msgs/MetricFamily.h"

namespace cartographer_ros_msgs
{

static const char READMETRICS[] = "cartographer_ros_msgs/ReadMetrics";

  class ReadMetricsRequest : public ros::Msg
  {
    public:

    ReadMetricsRequest()
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

    virtual const char * getType() override { return READMETRICS; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ReadMetricsResponse : public ros::Msg
  {
    public:
      typedef cartographer_ros_msgs::StatusResponse _status_type;
      _status_type status;
      uint32_t metric_families_length;
      typedef cartographer_ros_msgs::MetricFamily _metric_families_type;
      _metric_families_type st_metric_families;
      _metric_families_type * metric_families;
      typedef ros::Time _timestamp_type;
      _timestamp_type timestamp;

    ReadMetricsResponse():
      status(),
      metric_families_length(0), st_metric_families(), metric_families(nullptr),
      timestamp()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->status.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->metric_families_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->metric_families_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->metric_families_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->metric_families_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->metric_families_length);
      for( uint32_t i = 0; i < metric_families_length; i++){
      offset += this->metric_families[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->timestamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timestamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timestamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timestamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timestamp.sec);
      *(outbuffer + offset + 0) = (this->timestamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timestamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timestamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timestamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timestamp.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->status.deserialize(inbuffer + offset);
      uint32_t metric_families_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      metric_families_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      metric_families_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      metric_families_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->metric_families_length);
      if(metric_families_lengthT > metric_families_length)
        this->metric_families = (cartographer_ros_msgs::MetricFamily*)realloc(this->metric_families, metric_families_lengthT * sizeof(cartographer_ros_msgs::MetricFamily));
      metric_families_length = metric_families_lengthT;
      for( uint32_t i = 0; i < metric_families_length; i++){
      offset += this->st_metric_families.deserialize(inbuffer + offset);
        memcpy( &(this->metric_families[i]), &(this->st_metric_families), sizeof(cartographer_ros_msgs::MetricFamily));
      }
      this->timestamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->timestamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timestamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timestamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timestamp.sec);
      this->timestamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->timestamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timestamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timestamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timestamp.nsec);
     return offset;
    }

    virtual const char * getType() override { return READMETRICS; };
    virtual const char * getMD5() override { return "a1fe8d7dcf3708e96e015774b1df470e"; };

  };

  class ReadMetrics {
    public:
    typedef ReadMetricsRequest Request;
    typedef ReadMetricsResponse Response;
  };

}
#endif
