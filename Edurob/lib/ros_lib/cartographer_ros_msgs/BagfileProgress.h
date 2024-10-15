#ifndef _ROS_cartographer_ros_msgs_BagfileProgress_h
#define _ROS_cartographer_ros_msgs_BagfileProgress_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace cartographer_ros_msgs
{

  class BagfileProgress : public ros::Msg
  {
    public:
      typedef const char* _current_bagfile_name_type;
      _current_bagfile_name_type current_bagfile_name;
      typedef uint32_t _current_bagfile_id_type;
      _current_bagfile_id_type current_bagfile_id;
      typedef uint32_t _total_bagfiles_type;
      _total_bagfiles_type total_bagfiles;
      typedef uint32_t _total_messages_type;
      _total_messages_type total_messages;
      typedef uint32_t _processed_messages_type;
      _processed_messages_type processed_messages;
      typedef float _total_seconds_type;
      _total_seconds_type total_seconds;
      typedef float _processed_seconds_type;
      _processed_seconds_type processed_seconds;

    BagfileProgress():
      current_bagfile_name(""),
      current_bagfile_id(0),
      total_bagfiles(0),
      total_messages(0),
      processed_messages(0),
      total_seconds(0),
      processed_seconds(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_current_bagfile_name = strlen(this->current_bagfile_name);
      varToArr(outbuffer + offset, length_current_bagfile_name);
      offset += 4;
      memcpy(outbuffer + offset, this->current_bagfile_name, length_current_bagfile_name);
      offset += length_current_bagfile_name;
      *(outbuffer + offset + 0) = (this->current_bagfile_id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->current_bagfile_id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->current_bagfile_id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->current_bagfile_id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_bagfile_id);
      *(outbuffer + offset + 0) = (this->total_bagfiles >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->total_bagfiles >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->total_bagfiles >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->total_bagfiles >> (8 * 3)) & 0xFF;
      offset += sizeof(this->total_bagfiles);
      *(outbuffer + offset + 0) = (this->total_messages >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->total_messages >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->total_messages >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->total_messages >> (8 * 3)) & 0xFF;
      offset += sizeof(this->total_messages);
      *(outbuffer + offset + 0) = (this->processed_messages >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->processed_messages >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->processed_messages >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->processed_messages >> (8 * 3)) & 0xFF;
      offset += sizeof(this->processed_messages);
      union {
        float real;
        uint32_t base;
      } u_total_seconds;
      u_total_seconds.real = this->total_seconds;
      *(outbuffer + offset + 0) = (u_total_seconds.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_total_seconds.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_total_seconds.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_total_seconds.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->total_seconds);
      union {
        float real;
        uint32_t base;
      } u_processed_seconds;
      u_processed_seconds.real = this->processed_seconds;
      *(outbuffer + offset + 0) = (u_processed_seconds.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_processed_seconds.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_processed_seconds.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_processed_seconds.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->processed_seconds);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_current_bagfile_name;
      arrToVar(length_current_bagfile_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_current_bagfile_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_current_bagfile_name-1]=0;
      this->current_bagfile_name = (char *)(inbuffer + offset-1);
      offset += length_current_bagfile_name;
      this->current_bagfile_id =  ((uint32_t) (*(inbuffer + offset)));
      this->current_bagfile_id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->current_bagfile_id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->current_bagfile_id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->current_bagfile_id);
      this->total_bagfiles =  ((uint32_t) (*(inbuffer + offset)));
      this->total_bagfiles |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->total_bagfiles |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->total_bagfiles |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->total_bagfiles);
      this->total_messages =  ((uint32_t) (*(inbuffer + offset)));
      this->total_messages |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->total_messages |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->total_messages |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->total_messages);
      this->processed_messages =  ((uint32_t) (*(inbuffer + offset)));
      this->processed_messages |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->processed_messages |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->processed_messages |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->processed_messages);
      union {
        float real;
        uint32_t base;
      } u_total_seconds;
      u_total_seconds.base = 0;
      u_total_seconds.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_total_seconds.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_total_seconds.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_total_seconds.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->total_seconds = u_total_seconds.real;
      offset += sizeof(this->total_seconds);
      union {
        float real;
        uint32_t base;
      } u_processed_seconds;
      u_processed_seconds.base = 0;
      u_processed_seconds.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_processed_seconds.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_processed_seconds.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_processed_seconds.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->processed_seconds = u_processed_seconds.real;
      offset += sizeof(this->processed_seconds);
     return offset;
    }

    virtual const char * getType() override { return "cartographer_ros_msgs/BagfileProgress"; };
    virtual const char * getMD5() override { return "2a36f93b13e2b297d45098a38cb00510"; };

  };

}
#endif
