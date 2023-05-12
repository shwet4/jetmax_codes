// Generated by gencpp from file remote_control/SetServoRequest.msg
// DO NOT EDIT!


#ifndef REMOTE_CONTROL_MESSAGE_SETSERVOREQUEST_H
#define REMOTE_CONTROL_MESSAGE_SETSERVOREQUEST_H


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
struct SetServoRequest_
{
  typedef SetServoRequest_<ContainerAllocator> Type;

  SetServoRequest_()
    : servo_id(0)
    , angle(0.0)
    , duration(0.0)  {
    }
  SetServoRequest_(const ContainerAllocator& _alloc)
    : servo_id(0)
    , angle(0.0)
    , duration(0.0)  {
  (void)_alloc;
    }



   typedef uint8_t _servo_id_type;
  _servo_id_type servo_id;

   typedef float _angle_type;
  _angle_type angle;

   typedef float _duration_type;
  _duration_type duration;





  typedef boost::shared_ptr< ::remote_control::SetServoRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::remote_control::SetServoRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetServoRequest_

typedef ::remote_control::SetServoRequest_<std::allocator<void> > SetServoRequest;

typedef boost::shared_ptr< ::remote_control::SetServoRequest > SetServoRequestPtr;
typedef boost::shared_ptr< ::remote_control::SetServoRequest const> SetServoRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::remote_control::SetServoRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::remote_control::SetServoRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::remote_control::SetServoRequest_<ContainerAllocator1> & lhs, const ::remote_control::SetServoRequest_<ContainerAllocator2> & rhs)
{
  return lhs.servo_id == rhs.servo_id &&
    lhs.angle == rhs.angle &&
    lhs.duration == rhs.duration;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::remote_control::SetServoRequest_<ContainerAllocator1> & lhs, const ::remote_control::SetServoRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace remote_control

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::remote_control::SetServoRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::remote_control::SetServoRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::remote_control::SetServoRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::remote_control::SetServoRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::remote_control::SetServoRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::remote_control::SetServoRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::remote_control::SetServoRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "671428050eb747126f5eed5e8d463e4c";
  }

  static const char* value(const ::remote_control::SetServoRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x671428050eb74712ULL;
  static const uint64_t static_value2 = 0x6f5eed5e8d463e4cULL;
};

template<class ContainerAllocator>
struct DataType< ::remote_control::SetServoRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "remote_control/SetServoRequest";
  }

  static const char* value(const ::remote_control::SetServoRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::remote_control::SetServoRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 servo_id\n"
"float32 angle\n"
"float32 duration\n"
;
  }

  static const char* value(const ::remote_control::SetServoRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::remote_control::SetServoRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.servo_id);
      stream.next(m.angle);
      stream.next(m.duration);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetServoRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::remote_control::SetServoRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::remote_control::SetServoRequest_<ContainerAllocator>& v)
  {
    s << indent << "servo_id: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.servo_id);
    s << indent << "angle: ";
    Printer<float>::stream(s, indent + "  ", v.angle);
    s << indent << "duration: ";
    Printer<float>::stream(s, indent + "  ", v.duration);
  }
};

} // namespace message_operations
} // namespace ros

#endif // REMOTE_CONTROL_MESSAGE_SETSERVOREQUEST_H
