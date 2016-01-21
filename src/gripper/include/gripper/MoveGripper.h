// Generated by gencpp from file gripper/MoveGripper.msg
// DO NOT EDIT!


#ifndef GRIPPER_MESSAGE_MOVEGRIPPER_H
#define GRIPPER_MESSAGE_MOVEGRIPPER_H

#include <ros/service_traits.h>


#include <gripper/MoveGripperRequest.h>
#include <gripper/MoveGripperResponse.h>


namespace gripper
{

struct MoveGripper
{

typedef MoveGripperRequest Request;
typedef MoveGripperResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct MoveGripper
} // namespace gripper


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::gripper::MoveGripper > {
  static const char* value()
  {
    return "e6e865a8e83cc84fa1ea3e028f496ffb";
  }

  static const char* value(const ::gripper::MoveGripper&) { return value(); }
};

template<>
struct DataType< ::gripper::MoveGripper > {
  static const char* value()
  {
    return "gripper/MoveGripper";
  }

  static const char* value(const ::gripper::MoveGripper&) { return value(); }
};


// service_traits::MD5Sum< ::gripper::MoveGripperRequest> should match 
// service_traits::MD5Sum< ::gripper::MoveGripper > 
template<>
struct MD5Sum< ::gripper::MoveGripperRequest>
{
  static const char* value()
  {
    return MD5Sum< ::gripper::MoveGripper >::value();
  }
  static const char* value(const ::gripper::MoveGripperRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::gripper::MoveGripperRequest> should match 
// service_traits::DataType< ::gripper::MoveGripper > 
template<>
struct DataType< ::gripper::MoveGripperRequest>
{
  static const char* value()
  {
    return DataType< ::gripper::MoveGripper >::value();
  }
  static const char* value(const ::gripper::MoveGripperRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::gripper::MoveGripperResponse> should match 
// service_traits::MD5Sum< ::gripper::MoveGripper > 
template<>
struct MD5Sum< ::gripper::MoveGripperResponse>
{
  static const char* value()
  {
    return MD5Sum< ::gripper::MoveGripper >::value();
  }
  static const char* value(const ::gripper::MoveGripperResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::gripper::MoveGripperResponse> should match 
// service_traits::DataType< ::gripper::MoveGripper > 
template<>
struct DataType< ::gripper::MoveGripperResponse>
{
  static const char* value()
  {
    return DataType< ::gripper::MoveGripper >::value();
  }
  static const char* value(const ::gripper::MoveGripperResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // GRIPPER_MESSAGE_MOVEGRIPPER_H