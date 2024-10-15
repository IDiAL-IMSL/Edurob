#ifndef _ROS_SERVICE_FinishTrajectory_h
#define _ROS_SERVICE_FinishTrajectory_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "cartographer_ros_msgs/StatusResponse.h"

namespace cartographer_ros_msgs
{

static const char FINISHTRAJECTORY[] = "cartographer_ros_msgs/FinishTrajectory";

  class FinishTrajectoryRequest : public ros::Msg
  {
    public:
      typedef int32_t _trajectory_id_type;
      _trajectory_id_type trajectory_id;

    FinishTrajectoryRequest():
      trajectory_id(0)
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
     return offset;
    }

    virtual const char * getType() override { return FINISHTRAJECTORY; };
    virtual const char * getMD5() override { return "6e190c4be941828bcd09ea05053f4bb5"; };

  };

  class FinishTrajectoryResponse : public ros::Msg
  {
    public:
      typedef cartographer_ros_msgs::StatusResponse _status_type;
      _status_type status;

    FinishTrajectoryResponse():
      status()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->status.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->status.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return FINISHTRAJECTORY; };
    virtual const char * getMD5() override { return "4e6ca4e44081fa06b258fa12804ea7cb"; };

  };

  class FinishTrajectory {
    public:
    typedef FinishTrajectoryRequest Request;
    typedef FinishTrajectoryResponse Response;
  };

}
#endif
