// Generated by gencpp from file supervisor/GripperResponse.msg
// DO NOT EDIT!


#ifndef SUPERVISOR_MESSAGE_GRIPPERRESPONSE_H
#define SUPERVISOR_MESSAGE_GRIPPERRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace supervisor
{
template <class ContainerAllocator>
struct GripperResponse_
{
  typedef GripperResponse_<ContainerAllocator> Type;

  GripperResponse_()
    : succeeded(false)
    , error()  {
    }
  GripperResponse_(const ContainerAllocator& _alloc)
    : succeeded(false)
    , error(_alloc)  {
    }



   typedef uint8_t _succeeded_type;
  _succeeded_type succeeded;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _error_type;
  _error_type error;




  typedef boost::shared_ptr< ::supervisor::GripperResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::supervisor::GripperResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GripperResponse_

typedef ::supervisor::GripperResponse_<std::allocator<void> > GripperResponse;

typedef boost::shared_ptr< ::supervisor::GripperResponse > GripperResponsePtr;
typedef boost::shared_ptr< ::supervisor::GripperResponse const> GripperResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::supervisor::GripperResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::supervisor::GripperResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace supervisor

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'supervisor': ['/home/sanderkempen/Coding/Tesseron/src/supervisor/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::supervisor::GripperResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::supervisor::GripperResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::supervisor::GripperResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::supervisor::GripperResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::supervisor::GripperResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::supervisor::GripperResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::supervisor::GripperResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5f98c37a16a9673a5429ccb0e5cd31a8";
  }

  static const char* value(const ::supervisor::GripperResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5f98c37a16a9673aULL;
  static const uint64_t static_value2 = 0x5429ccb0e5cd31a8ULL;
};

template<class ContainerAllocator>
struct DataType< ::supervisor::GripperResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "supervisor/GripperResponse";
  }

  static const char* value(const ::supervisor::GripperResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::supervisor::GripperResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool succeeded\n\
string error\n\
\n\
";
  }

  static const char* value(const ::supervisor::GripperResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::supervisor::GripperResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.succeeded);
      stream.next(m.error);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct GripperResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::supervisor::GripperResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::supervisor::GripperResponse_<ContainerAllocator>& v)
  {
    s << indent << "succeeded: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.succeeded);
    s << indent << "error: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.error);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SUPERVISOR_MESSAGE_GRIPPERRESPONSE_H