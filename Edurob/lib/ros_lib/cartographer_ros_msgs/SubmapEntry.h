#ifndef _ROS_cartographer_ros_msgs_SubmapEntry_h
#define _ROS_cartographer_ros_msgs_SubmapEntry_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"

namespace cartographer_ros_msgs
{

  class SubmapEntry : public ros::Msg
  {
    public:
      typedef int32_t _trajectory_id_type;
      _trajectory_id_type trajectory_id;
      typedef int32_t _submap_index_type;
      _submap_index_type submap_index;
      typedef int32_t _submap_version_type;
      _submap_version_type submap_version;
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;
      typedef bool _is_frozen_type;
      _is_frozen_type is_frozen;

    SubmapEntry():
      trajectory_id(0),
      submap_index(0),
      submap_version(0),
      pose(),
      is_frozen(0)
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
      offset += this->pose.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_is_frozen;
      u_is_frozen.real = this->is_frozen;
      *(outbuffer + offset + 0) = (u_is_frozen.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_frozen);
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
      offset += this->pose.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_is_frozen;
      u_is_frozen.base = 0;
      u_is_frozen.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_frozen = u_is_frozen.real;
      offset += sizeof(this->is_frozen);
     return offset;
    }

    virtual const char * getType() override { return "cartographer_ros_msgs/SubmapEntry"; };
    virtual const char * getMD5() override { return "0b064ac06b4af2be11388441508c9572"; };

  };

}
#endif
