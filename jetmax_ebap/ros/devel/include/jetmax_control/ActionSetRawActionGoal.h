// Generated by gencpp from file jetmax_control/ActionSetRawActionGoal.msg
// DO NOT EDIT!


#ifndef JETMAX_CONTROL_MESSAGE_ACTIONSETRAWACTIONGOAL_H
#define JETMAX_CONTROL_MESSAGE_ACTIONSETRAWACTIONGOAL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <actionlib_msgs/GoalID.h>
#include <jetmax_control/ActionSetRawGoal.h>

namespace jetmax_control
{
template <class ContainerAllocator>
struct ActionSetRawActionGoal_
{
  typedef ActionSetRawActionGoal_<ContainerAllocator> Type;

  ActionSetRawActionGoal_()
    : header()
    , goal_id()
    , goal()  {
    }
  ActionSetRawActionGoal_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , goal_id(_alloc)
    , goal(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::actionlib_msgs::GoalID_<ContainerAllocator>  _goal_id_type;
  _goal_id_type goal_id;

   typedef  ::jetmax_control::ActionSetRawGoal_<ContainerAllocator>  _goal_type;
  _goal_type goal;





  typedef boost::shared_ptr< ::jetmax_control::ActionSetRawActionGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jetmax_control::ActionSetRawActionGoal_<ContainerAllocator> const> ConstPtr;

}; // struct ActionSetRawActionGoal_

typedef ::jetmax_control::ActionSetRawActionGoal_<std::allocator<void> > ActionSetRawActionGoal;

typedef boost::shared_ptr< ::jetmax_control::ActionSetRawActionGoal > ActionSetRawActionGoalPtr;
typedef boost::shared_ptr< ::jetmax_control::ActionSetRawActionGoal const> ActionSetRawActionGoalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jetmax_control::ActionSetRawActionGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jetmax_control::ActionSetRawActionGoal_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::jetmax_control::ActionSetRawActionGoal_<ContainerAllocator1> & lhs, const ::jetmax_control::ActionSetRawActionGoal_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.goal_id == rhs.goal_id &&
    lhs.goal == rhs.goal;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::jetmax_control::ActionSetRawActionGoal_<ContainerAllocator1> & lhs, const ::jetmax_control::ActionSetRawActionGoal_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace jetmax_control

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::jetmax_control::ActionSetRawActionGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jetmax_control::ActionSetRawActionGoal_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jetmax_control::ActionSetRawActionGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jetmax_control::ActionSetRawActionGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jetmax_control::ActionSetRawActionGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jetmax_control::ActionSetRawActionGoal_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jetmax_control::ActionSetRawActionGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b88dbb0562bfa6d5322403016e66889d";
  }

  static const char* value(const ::jetmax_control::ActionSetRawActionGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb88dbb0562bfa6d5ULL;
  static const uint64_t static_value2 = 0x322403016e66889dULL;
};

template<class ContainerAllocator>
struct DataType< ::jetmax_control::ActionSetRawActionGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jetmax_control/ActionSetRawActionGoal";
  }

  static const char* value(const ::jetmax_control::ActionSetRawActionGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jetmax_control::ActionSetRawActionGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"Header header\n"
"actionlib_msgs/GoalID goal_id\n"
"ActionSetRawGoal goal\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: actionlib_msgs/GoalID\n"
"# The stamp should store the time at which this goal was requested.\n"
"# It is used by an action server when it tries to preempt all\n"
"# goals that were requested before a certain time\n"
"time stamp\n"
"\n"
"# The id provides a way to associate feedback and\n"
"# result message with specific goal requests. The id\n"
"# specified must be unique.\n"
"string id\n"
"\n"
"\n"
"================================================================================\n"
"MSG: jetmax_control/ActionSetRawGoal\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"string data\n"
"uint32 repeat\n"
;
  }

  static const char* value(const ::jetmax_control::ActionSetRawActionGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jetmax_control::ActionSetRawActionGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.goal_id);
      stream.next(m.goal);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ActionSetRawActionGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jetmax_control::ActionSetRawActionGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jetmax_control::ActionSetRawActionGoal_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "goal_id: ";
    s << std::endl;
    Printer< ::actionlib_msgs::GoalID_<ContainerAllocator> >::stream(s, indent + "  ", v.goal_id);
    s << indent << "goal: ";
    s << std::endl;
    Printer< ::jetmax_control::ActionSetRawGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.goal);
  }
};

} // namespace message_operations
} // namespace ros

#endif // JETMAX_CONTROL_MESSAGE_ACTIONSETRAWACTIONGOAL_H
