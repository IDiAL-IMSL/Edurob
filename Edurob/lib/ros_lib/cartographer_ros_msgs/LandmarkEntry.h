#ifndef _ROS_cartographer_ros_msgs_LandmarkEntry_h
#define _ROS_cartographer_ros_msgs_LandmarkEntry_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"

namespace cartographer_ros_msgs
{

  class LandmarkEntry : public ros::Msg
  {
    public:
      typedef const char* _id_type;
      _id_type id;
      typedef geometry_msgs::Pose _tracking_from_landmark_transform_type;
      _tracking_from_landmark_transform_type tracking_from_landmark_transform;
      typedef float _translation_weight_type;
      _translation_weight_type translation_weight;
      typedef float _rotation_weight_type;
      _rotation_weight_type rotation_weight;

    LandmarkEntry():
      id(""),
      tracking_from_landmark_transform(),
      translation_weight(0),
      rotation_weight(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_id = strlen(this->id);
      varToArr(outbuffer + offset, length_id);
      offset += 4;
      memcpy(outbuffer + offset, this->id, length_id);
      offset += length_id;
      offset += this->tracking_from_landmark_transform.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->translation_weight);
      offset += serializeAvrFloat64(outbuffer + offset, this->rotation_weight);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_id;
      arrToVar(length_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_id-1]=0;
      this->id = (char *)(inbuffer + offset-1);
      offset += length_id;
      offset += this->tracking_from_landmark_transform.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->translation_weight));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->rotation_weight));
     return offset;
    }

    virtual const char * getType() override { return "cartographer_ros_msgs/LandmarkEntry"; };
    virtual const char * getMD5() override { return "133f8dd7259f83a87eb4d78b301c0b70"; };

  };

}
#endif
