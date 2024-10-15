#ifndef _ROS_cartographer_ros_msgs_Metric_h
#define _ROS_cartographer_ros_msgs_Metric_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "cartographer_ros_msgs/MetricLabel.h"
#include "cartographer_ros_msgs/HistogramBucket.h"

namespace cartographer_ros_msgs
{

  class Metric : public ros::Msg
  {
    public:
      typedef uint8_t _type_type;
      _type_type type;
      uint32_t labels_length;
      typedef cartographer_ros_msgs::MetricLabel _labels_type;
      _labels_type st_labels;
      _labels_type * labels;
      typedef float _value_type;
      _value_type value;
      uint32_t counts_by_bucket_length;
      typedef cartographer_ros_msgs::HistogramBucket _counts_by_bucket_type;
      _counts_by_bucket_type st_counts_by_bucket;
      _counts_by_bucket_type * counts_by_bucket;
      enum { TYPE_COUNTER = 0 };
      enum { TYPE_GAUGE = 1 };
      enum { TYPE_HISTOGRAM = 2 };

    Metric():
      type(0),
      labels_length(0), st_labels(), labels(nullptr),
      value(0),
      counts_by_bucket_length(0), st_counts_by_bucket(), counts_by_bucket(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->type);
      *(outbuffer + offset + 0) = (this->labels_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->labels_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->labels_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->labels_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->labels_length);
      for( uint32_t i = 0; i < labels_length; i++){
      offset += this->labels[i].serialize(outbuffer + offset);
      }
      offset += serializeAvrFloat64(outbuffer + offset, this->value);
      *(outbuffer + offset + 0) = (this->counts_by_bucket_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->counts_by_bucket_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->counts_by_bucket_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->counts_by_bucket_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->counts_by_bucket_length);
      for( uint32_t i = 0; i < counts_by_bucket_length; i++){
      offset += this->counts_by_bucket[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->type);
      uint32_t labels_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      labels_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      labels_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      labels_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->labels_length);
      if(labels_lengthT > labels_length)
        this->labels = (cartographer_ros_msgs::MetricLabel*)realloc(this->labels, labels_lengthT * sizeof(cartographer_ros_msgs::MetricLabel));
      labels_length = labels_lengthT;
      for( uint32_t i = 0; i < labels_length; i++){
      offset += this->st_labels.deserialize(inbuffer + offset);
        memcpy( &(this->labels[i]), &(this->st_labels), sizeof(cartographer_ros_msgs::MetricLabel));
      }
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->value));
      uint32_t counts_by_bucket_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      counts_by_bucket_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      counts_by_bucket_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      counts_by_bucket_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->counts_by_bucket_length);
      if(counts_by_bucket_lengthT > counts_by_bucket_length)
        this->counts_by_bucket = (cartographer_ros_msgs::HistogramBucket*)realloc(this->counts_by_bucket, counts_by_bucket_lengthT * sizeof(cartographer_ros_msgs::HistogramBucket));
      counts_by_bucket_length = counts_by_bucket_lengthT;
      for( uint32_t i = 0; i < counts_by_bucket_length; i++){
      offset += this->st_counts_by_bucket.deserialize(inbuffer + offset);
        memcpy( &(this->counts_by_bucket[i]), &(this->st_counts_by_bucket), sizeof(cartographer_ros_msgs::HistogramBucket));
      }
     return offset;
    }

    virtual const char * getType() override { return "cartographer_ros_msgs/Metric"; };
    virtual const char * getMD5() override { return "94a6ea1c6ef245b483a220f6769c8e36"; };

  };

}
#endif
