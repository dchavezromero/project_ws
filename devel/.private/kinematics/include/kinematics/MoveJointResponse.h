// Generated by gencpp from file kinematics/MoveJointResponse.msg
// DO NOT EDIT!


#ifndef KINEMATICS_MESSAGE_MOVEJOINTRESPONSE_H
#define KINEMATICS_MESSAGE_MOVEJOINTRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace kinematics
{
template <class ContainerAllocator>
struct MoveJointResponse_
{
  typedef MoveJointResponse_<ContainerAllocator> Type;

  MoveJointResponse_()
    : valid_position(false)  {
    }
  MoveJointResponse_(const ContainerAllocator& _alloc)
    : valid_position(false)  {
  (void)_alloc;
    }



   typedef uint8_t _valid_position_type;
  _valid_position_type valid_position;





  typedef boost::shared_ptr< ::kinematics::MoveJointResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kinematics::MoveJointResponse_<ContainerAllocator> const> ConstPtr;

}; // struct MoveJointResponse_

typedef ::kinematics::MoveJointResponse_<std::allocator<void> > MoveJointResponse;

typedef boost::shared_ptr< ::kinematics::MoveJointResponse > MoveJointResponsePtr;
typedef boost::shared_ptr< ::kinematics::MoveJointResponse const> MoveJointResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kinematics::MoveJointResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kinematics::MoveJointResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kinematics::MoveJointResponse_<ContainerAllocator1> & lhs, const ::kinematics::MoveJointResponse_<ContainerAllocator2> & rhs)
{
  return lhs.valid_position == rhs.valid_position;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kinematics::MoveJointResponse_<ContainerAllocator1> & lhs, const ::kinematics::MoveJointResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kinematics

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::kinematics::MoveJointResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kinematics::MoveJointResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kinematics::MoveJointResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kinematics::MoveJointResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kinematics::MoveJointResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kinematics::MoveJointResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kinematics::MoveJointResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1d7edd7f640319e72564c9ef71c5afb3";
  }

  static const char* value(const ::kinematics::MoveJointResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1d7edd7f640319e7ULL;
  static const uint64_t static_value2 = 0x2564c9ef71c5afb3ULL;
};

template<class ContainerAllocator>
struct DataType< ::kinematics::MoveJointResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kinematics/MoveJointResponse";
  }

  static const char* value(const ::kinematics::MoveJointResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kinematics::MoveJointResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool valid_position\n"
;
  }

  static const char* value(const ::kinematics::MoveJointResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kinematics::MoveJointResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.valid_position);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MoveJointResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kinematics::MoveJointResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kinematics::MoveJointResponse_<ContainerAllocator>& v)
  {
    s << indent << "valid_position: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.valid_position);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KINEMATICS_MESSAGE_MOVEJOINTRESPONSE_H