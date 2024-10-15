#ifndef _ROS_cartographer_ros_msgs_SubmapList_h
#define _ROS_cartographer_ros_msgs_SubmapList_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "cartographer_ros_msgs/SubmapEntry.h"

namespace cartographer_ros_msgs
{

  class SubmapList : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t submap_length;
      typedef cartographer_ros_msgs::SubmapEntry _submap_type;
      _submap_type st_submap;
      _submap_type * submap;

    SubmapList():
      header(),
      submap_length(0), st_submap(), submap(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->submap_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->submap_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->submap_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->submap_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->submap_length);
      for( uint32_t i = 0; i < submap_length; i++){
      offset += this->submap[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t submap_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      submap_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      submap_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      submap_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->submap_length);
      if(submap_lengthT > submap_length)
        this->submap = (cartographer_ros_msgs::SubmapEntry*)realloc(this->submap, submap_lengthT * sizeof(cartographer_ros_msgs::SubmapEntry));
      submap_length = submap_lengthT;
      for( uint32_t i = 0; i < submap_length; i++){
      offset += this->st_submap.deserialize(inbuffer + offset);
        memcpy( &(this->submap[i]), &(this->st_submap), sizeof(cartographer_ros_msgs::SubmapEntry));
      }
     return offset;
    }

    virtual const char * getType() override { return "cartographer_ros_msgs/SubmapList"; };
    virtual const char * getMD5() override { return "73b1e412208f0787050395996f6143db"; };

  };

}
#endif
