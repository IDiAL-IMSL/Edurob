#ifndef _ROS_SERVICE_GetTrajectoryStates_h
#define _ROS_SERVICE_GetTrajectoryStates_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "cartographer_ros_msgs/StatusResponse.h"
#include "cartographer_ros_msgs/TrajectoryStates.h"

namespace cartographer_ros_msgs
{

static const char GETTRAJECTORYSTATES[] = "cartographer_ros_msgs/GetTrajectoryStates";

  class GetTrajectoryStatesRequest : public ros::Msg
  {
    public:

    GetTrajectoryStatesRequest()
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

    virtual const char * getType() override { return GETTRAJECTORYSTATES; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetTrajectoryStatesResponse : public ros::Msg
  {
    public:
      typedef cartographer_ros_msgs::StatusResponse _status_type;
      _status_type status;
      typedef cartographer_ros_msgs::TrajectoryStates _trajectory_states_type;
      _trajectory_states_type trajectory_states;

    GetTrajectoryStatesResponse():
      status(),
      trajectory_states()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->status.serialize(outbuffer + offset);
      offset += this->trajectory_states.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->trajectory_states.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return GETTRAJECTORYSTATES; };
    virtual const char * getMD5() override { return "b9e3b373f17df088ee6dcd817b79dff0"; };

  };

  class GetTrajectoryStates {
    public:
    typedef GetTrajectoryStatesRequest Request;
    typedef GetTrajectoryStatesResponse Response;
  };

}
#endif
