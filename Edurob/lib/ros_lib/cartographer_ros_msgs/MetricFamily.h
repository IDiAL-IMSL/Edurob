#ifndef _ROS_cartographer_ros_msgs_MetricFamily_h
#define _ROS_cartographer_ros_msgs_MetricFamily_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "cartographer_ros_msgs/Metric.h"

namespace cartographer_ros_msgs
{

  class MetricFamily : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _description_type;
      _description_type description;
      uint32_t metrics_length;
      typedef cartographer_ros_msgs::Metric _metrics_type;
      _metrics_type st_metrics;
      _metrics_type * metrics;

    MetricFamily():
      name(""),
      description(""),
      metrics_length(0), st_metrics(), metrics(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      uint32_t length_description = strlen(this->description);
      varToArr(outbuffer + offset, length_description);
      offset += 4;
      memcpy(outbuffer + offset, this->description, length_description);
      offset += length_description;
      *(outbuffer + offset + 0) = (this->metrics_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->metrics_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->metrics_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->metrics_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->metrics_length);
      for( uint32_t i = 0; i < metrics_length; i++){
      offset += this->metrics[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint32_t length_description;
      arrToVar(length_description, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_description; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_description-1]=0;
      this->description = (char *)(inbuffer + offset-1);
      offset += length_description;
      uint32_t metrics_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      metrics_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      metrics_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      metrics_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->metrics_length);
      if(metrics_lengthT > metrics_length)
        this->metrics = (cartographer_ros_msgs::Metric*)realloc(this->metrics, metrics_lengthT * sizeof(cartographer_ros_msgs::Metric));
      metrics_length = metrics_lengthT;
      for( uint32_t i = 0; i < metrics_length; i++){
      offset += this->st_metrics.deserialize(inbuffer + offset);
        memcpy( &(this->metrics[i]), &(this->st_metrics), sizeof(cartographer_ros_msgs::Metric));
      }
     return offset;
    }

    virtual const char * getType() override { return "cartographer_ros_msgs/MetricFamily"; };
    virtual const char * getMD5() override { return "583a11b161bb4a70f5df274715bcaf10"; };

  };

}
#endif
