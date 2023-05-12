// Generated by gencpp from file remote_control/SetPositionRequest.msg
// DO NOT EDIT!


#ifndef REMOTE_CONTROL_MESSAGE_SETPOSITIONREQUEST_H
#define REMOTE_CONTROL_MESSAGE_SETPOSITIONREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace remote_control
{
template <class ContainerAllocator>
struct SetPositionRequest_
{
  typedef SetPositionRequest_<ContainerAllocator> Type;

  SetPositionRequest_()
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , duration(0.0)  {
    }
  SetPositionRequest_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , duration(0.0)  {
  (void)_alloc;
    }



   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _z_type;
  _z_type z;

   typedef float _duration_type;
  _duration_type duration;





  typedef boost::shared_ptr< ::remote_control::SetPositionRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::remote_control::SetPositionRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetPositionRequest_

typedef ::remote_control::SetPositionRequest_<std::allocator<void> > SetPositionRequest;

typedef boost::shared_ptr< ::remote_control::SetPositionRequest > SetPositionRequestPtr;
typedef boost::shared_ptr< ::remote_control::SetPositionRequest const> SetPositionRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::remote_control::SetPositionRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::remote_control::SetPositionRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::remote_control::SetPositionRequest_<ContainerAllocator1> & lhs, const ::remote_control::SetPositionRequest_<ContainerAllocator2> & rhs)
{
  return lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.z == rhs.z &&
    lhs.duration == rhs.duration;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::remote_control::SetPositionRequest_<ContainerAllocator1> & lhs, const ::remote_control::SetPositionRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace remote_control

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::remote_control::SetPositionRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::remote_control::SetPositionRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::remote_control::SetPositionRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::remote_control::SetPositionRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::remote_control::SetPositionRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::remote_control::SetPositionRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::remote_control::SetPositionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "19cc8078d43cb0cd983e9688d658cb05";
  }

  static const char* value(const ::remote_control::SetPositionRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x19cc8078d43cb0cdULL;
  static const uint64_t static_value2 = 0x983e9688d658cb05ULL;
};

template<class ContainerAllocator>
struct DataType< ::remote_control::SetPositionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "remote_control/SetPositionRequest";
  }

  static const char* value(const ::remote_control::SetPositionRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::remote_control::SetPositionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 x\n"
"float32 y\n"
"float32 z\n"
"float32 duration\n"
;
  }

  static const char* value(const ::remote_control::SetPositionRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::remote_control::SetPositionRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
      stream.next(m.duration);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetPositionRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::remote_control::SetPositionRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::remote_control::SetPositionRequest_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<float>::stream(s, indent + "  ", v.z);
    s << indent << "duration: ";
    Printer<float>::stream(s, indent + "  ", v.duration);
  }
};

} // namespace message_operations
} // namespace ros

#endif // REMOTE_CONTROL_MESSAGE_SETPOSITIONREQUEST_H
