#ifndef _ROS_SERVICE_SubmapQuery_h
#define _ROS_SERVICE_SubmapQuery_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "cartographer_ros_msgs/StatusResponse.h"
#include "cartographer_ros_msgs/SubmapTexture.h"

namespace cartographer_ros_msgs
{

static const char SUBMAPQUERY[] = "cartographer_ros_msgs/SubmapQuery";

  class SubmapQueryRequest : public ros::Msg
  {
    public:
      typedef int32_t _trajectory_id_type;
      _trajectory_id_type trajectory_id;
      typedef int32_t _submap_index_type;
      _submap_index_type submap_index;

    SubmapQueryRequest():
      trajectory_id(0),
      submap_index(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_trajectory_id;
      u_trajectory_id.real = this->trajectory_id;
      *(outbuffer + offset + 0) = (u_trajectory_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_trajectory_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_trajectory_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_trajectory_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->trajectory_id);
      union {
        int32_t real;
        uint32_t base;
      } u_submap_index;
      u_submap_index.real = this->submap_index;
      *(outbuffer + offset + 0) = (u_submap_index.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_submap_index.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_submap_index.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_submap_index.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->submap_index);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_trajectory_id;
      u_trajectory_id.base = 0;
      u_trajectory_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_trajectory_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_trajectory_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_trajectory_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->trajectory_id = u_trajectory_id.real;
      offset += sizeof(this->trajectory_id);
      union {
        int32_t real;
        uint32_t base;
      } u_submap_index;
      u_submap_index.base = 0;
      u_submap_index.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_submap_index.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_submap_index.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_submap_index.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->submap_index = u_submap_index.real;
      offset += sizeof(this->submap_index);
     return offset;
    }

    virtual const char * getType() override { return SUBMAPQUERY; };
    virtual const char * getMD5() override { return "5fc429a478a6d73822616720a31a2158"; };

  };

  class SubmapQueryResponse : public ros::Msg
  {
    public:
      typedef cartographer_ros_msgs::StatusResponse _status_type;
      _status_type status;
      typedef int32_t _submap_version_type;
      _submap_version_type submap_version;
      uint32_t textures_length;
      typedef cartographer_ros_msgs::SubmapTexture _textures_type;
      _textures_type st_textures;
      _textures_type * textures;

    SubmapQueryResponse():
      status(),
      submap_version(0),
      textures_length(0), st_textures(), textures(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->status.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_submap_version;
      u_submap_version.real = this->submap_version;
      *(outbuffer + offset + 0) = (u_submap_version.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_submap_version.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_submap_version.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_submap_version.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->submap_version);
      *(outbuffer + offset + 0) = (this->textures_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->textures_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->textures_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->textures_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->textures_length);
      for( uint32_t i = 0; i < textures_length; i++){
      offset += this->textures[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->status.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_submap_version;
      u_submap_version.base = 0;
      u_submap_version.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_submap_version.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_submap_version.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_submap_version.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->submap_version = u_submap_version.real;
      offset += sizeof(this->submap_version);
      uint32_t textures_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      textures_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      textures_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      textures_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->textures_length);
      if(textures_lengthT > textures_length)
        this->textures = (cartographer_ros_msgs::SubmapTexture*)realloc(this->textures, textures_lengthT * sizeof(cartographer_ros_msgs::SubmapTexture));
      textures_length = textures_lengthT;
      for( uint32_t i = 0; i < textures_length; i++){
      offset += this->st_textures.deserialize(inbuffer + offset);
        memcpy( &(this->textures[i]), &(this->st_textures), sizeof(cartographer_ros_msgs::SubmapTexture));
      }
     return offset;
    }

    virtual const char * getType() override { return SUBMAPQUERY; };
    virtual const char * getMD5() override { return "ffc82c14b81fa551bc249c31ba402b2e"; };

  };

  class SubmapQuery {
    public:
    typedef SubmapQueryRequest Request;
    typedef SubmapQueryResponse Response;
  };

}
#endif
