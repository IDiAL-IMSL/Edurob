#ifndef _ROS_SERVICE_StartTrajectory_h
#define _ROS_SERVICE_StartTrajectory_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"
#include "cartographer_ros_msgs/StatusResponse.h"

namespace cartographer_ros_msgs
{

static const char STARTTRAJECTORY[] = "cartographer_ros_msgs/StartTrajectory";

  class StartTrajectoryRequest : public ros::Msg
  {
    public:
      typedef const char* _configuration_directory_type;
      _configuration_directory_type configuration_directory;
      typedef const char* _configuration_basename_type;
      _configuration_basename_type configuration_basename;
      typedef bool _use_initial_pose_type;
      _use_initial_pose_type use_initial_pose;
      typedef geometry_msgs::Pose _initial_pose_type;
      _initial_pose_type initial_pose;
      typedef int32_t _relative_to_trajectory_id_type;
      _relative_to_trajectory_id_type relative_to_trajectory_id;

    StartTrajectoryRequest():
      configuration_directory(""),
      configuration_basename(""),
      use_initial_pose(0),
      initial_pose(),
      relative_to_trajectory_id(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_configuration_directory = strlen(this->configuration_directory);
      varToArr(outbuffer + offset, length_configuration_directory);
      offset += 4;
      memcpy(outbuffer + offset, this->configuration_directory, length_configuration_directory);
      offset += length_configuration_directory;
      uint32_t length_configuration_basename = strlen(this->configuration_basename);
      varToArr(outbuffer + offset, length_configuration_basename);
      offset += 4;
      memcpy(outbuffer + offset, this->configuration_basename, length_configuration_basename);
      offset += length_configuration_basename;
      union {
        bool real;
        uint8_t base;
      } u_use_initial_pose;
      u_use_initial_pose.real = this->use_initial_pose;
      *(outbuffer + offset + 0) = (u_use_initial_pose.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->use_initial_pose);
      offset += this->initial_pose.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_relative_to_trajectory_id;
      u_relative_to_trajectory_id.real = this->relative_to_trajectory_id;
      *(outbuffer + offset + 0) = (u_relative_to_trajectory_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_relative_to_trajectory_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_relative_to_trajectory_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_relative_to_trajectory_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->relative_to_trajectory_id);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_configuration_directory;
      arrToVar(length_configuration_directory, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_configuration_directory; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_configuration_directory-1]=0;
      this->configuration_directory = (char *)(inbuffer + offset-1);
      offset += length_configuration_directory;
      uint32_t length_configuration_basename;
      arrToVar(length_configuration_basename, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_configuration_basename; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_configuration_basename-1]=0;
      this->configuration_basename = (char *)(inbuffer + offset-1);
      offset += length_configuration_basename;
      union {
        bool real;
        uint8_t base;
      } u_use_initial_pose;
      u_use_initial_pose.base = 0;
      u_use_initial_pose.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->use_initial_pose = u_use_initial_pose.real;
      offset += sizeof(this->use_initial_pose);
      offset += this->initial_pose.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_relative_to_trajectory_id;
      u_relative_to_trajectory_id.base = 0;
      u_relative_to_trajectory_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_relative_to_trajectory_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_relative_to_trajectory_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_relative_to_trajectory_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->relative_to_trajectory_id = u_relative_to_trajectory_id.real;
      offset += sizeof(this->relative_to_trajectory_id);
     return offset;
    }

    virtual const char * getType() override { return STARTTRAJECTORY; };
    virtual const char * getMD5() override { return "555a1aa894dfd093eaa13b245b423df8"; };

  };

  class StartTrajectoryResponse : public ros::Msg
  {
    public:
      typedef cartographer_ros_msgs::StatusResponse _status_type;
      _status_type status;
      typedef int32_t _trajectory_id_type;
      _trajectory_id_type trajectory_id;

    StartTrajectoryResponse():
      status(),
      trajectory_id(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->status.serialize(outbuffer + offset);
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
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->status.deserialize(inbuffer + offset);
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
     return offset;
    }

    virtual const char * getType() override { return STARTTRAJECTORY; };
    virtual const char * getMD5() override { return "a14602d76d9b734b374a25be319cdbe9"; };

  };

  class StartTrajectory {
    public:
    typedef StartTrajectoryRequest Request;
    typedef StartTrajectoryResponse Response;
  };

}
#endif
