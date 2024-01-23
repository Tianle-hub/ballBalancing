// Generated by gencpp from file tum_ics_lacquey_gripper_msgs/setGripperState.msg
// DO NOT EDIT!


#ifndef TUM_ICS_LACQUEY_GRIPPER_MSGS_MESSAGE_SETGRIPPERSTATE_H
#define TUM_ICS_LACQUEY_GRIPPER_MSGS_MESSAGE_SETGRIPPERSTATE_H

#include <ros/service_traits.h>


#include <tum_ics_lacquey_gripper_msgs/setGripperStateRequest.h>
#include <tum_ics_lacquey_gripper_msgs/setGripperStateResponse.h>


namespace tum_ics_lacquey_gripper_msgs
{

struct setGripperState
{

typedef setGripperStateRequest Request;
typedef setGripperStateResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct setGripperState
} // namespace tum_ics_lacquey_gripper_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::tum_ics_lacquey_gripper_msgs::setGripperState > {
  static const char* value()
  {
    return "8223e23338c4551db84e86bb33c66825";
  }

  static const char* value(const ::tum_ics_lacquey_gripper_msgs::setGripperState&) { return value(); }
};

template<>
struct DataType< ::tum_ics_lacquey_gripper_msgs::setGripperState > {
  static const char* value()
  {
    return "tum_ics_lacquey_gripper_msgs/setGripperState";
  }

  static const char* value(const ::tum_ics_lacquey_gripper_msgs::setGripperState&) { return value(); }
};


// service_traits::MD5Sum< ::tum_ics_lacquey_gripper_msgs::setGripperStateRequest> should match
// service_traits::MD5Sum< ::tum_ics_lacquey_gripper_msgs::setGripperState >
template<>
struct MD5Sum< ::tum_ics_lacquey_gripper_msgs::setGripperStateRequest>
{
  static const char* value()
  {
    return MD5Sum< ::tum_ics_lacquey_gripper_msgs::setGripperState >::value();
  }
  static const char* value(const ::tum_ics_lacquey_gripper_msgs::setGripperStateRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::tum_ics_lacquey_gripper_msgs::setGripperStateRequest> should match
// service_traits::DataType< ::tum_ics_lacquey_gripper_msgs::setGripperState >
template<>
struct DataType< ::tum_ics_lacquey_gripper_msgs::setGripperStateRequest>
{
  static const char* value()
  {
    return DataType< ::tum_ics_lacquey_gripper_msgs::setGripperState >::value();
  }
  static const char* value(const ::tum_ics_lacquey_gripper_msgs::setGripperStateRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::tum_ics_lacquey_gripper_msgs::setGripperStateResponse> should match
// service_traits::MD5Sum< ::tum_ics_lacquey_gripper_msgs::setGripperState >
template<>
struct MD5Sum< ::tum_ics_lacquey_gripper_msgs::setGripperStateResponse>
{
  static const char* value()
  {
    return MD5Sum< ::tum_ics_lacquey_gripper_msgs::setGripperState >::value();
  }
  static const char* value(const ::tum_ics_lacquey_gripper_msgs::setGripperStateResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::tum_ics_lacquey_gripper_msgs::setGripperStateResponse> should match
// service_traits::DataType< ::tum_ics_lacquey_gripper_msgs::setGripperState >
template<>
struct DataType< ::tum_ics_lacquey_gripper_msgs::setGripperStateResponse>
{
  static const char* value()
  {
    return DataType< ::tum_ics_lacquey_gripper_msgs::setGripperState >::value();
  }
  static const char* value(const ::tum_ics_lacquey_gripper_msgs::setGripperStateResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // TUM_ICS_LACQUEY_GRIPPER_MSGS_MESSAGE_SETGRIPPERSTATE_H
