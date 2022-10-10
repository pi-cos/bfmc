// Generated by gencpp from file utils/environmental.msg
// DO NOT EDIT!


#ifndef UTILS_MESSAGE_ENVIRONMENTAL_H
#define UTILS_MESSAGE_ENVIRONMENTAL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace utils
{
template <class ContainerAllocator>
struct environmental_
{
  typedef environmental_<ContainerAllocator> Type;

  environmental_()
    : obstacle_id(0)
    , x(0.0)
    , y(0.0)  {
    }
  environmental_(const ContainerAllocator& _alloc)
    : obstacle_id(0)
    , x(0.0)
    , y(0.0)  {
  (void)_alloc;
    }



   typedef uint8_t _obstacle_id_type;
  _obstacle_id_type obstacle_id;

   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;





  typedef boost::shared_ptr< ::utils::environmental_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::utils::environmental_<ContainerAllocator> const> ConstPtr;

}; // struct environmental_

typedef ::utils::environmental_<std::allocator<void> > environmental;

typedef boost::shared_ptr< ::utils::environmental > environmentalPtr;
typedef boost::shared_ptr< ::utils::environmental const> environmentalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::utils::environmental_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::utils::environmental_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::utils::environmental_<ContainerAllocator1> & lhs, const ::utils::environmental_<ContainerAllocator2> & rhs)
{
  return lhs.obstacle_id == rhs.obstacle_id &&
    lhs.x == rhs.x &&
    lhs.y == rhs.y;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::utils::environmental_<ContainerAllocator1> & lhs, const ::utils::environmental_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace utils

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::utils::environmental_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::utils::environmental_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::utils::environmental_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::utils::environmental_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::utils::environmental_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::utils::environmental_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::utils::environmental_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a1acf3f1b0fd75ef5b1b19cde1c2ce7f";
  }

  static const char* value(const ::utils::environmental_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa1acf3f1b0fd75efULL;
  static const uint64_t static_value2 = 0x5b1b19cde1c2ce7fULL;
};

template<class ContainerAllocator>
struct DataType< ::utils::environmental_<ContainerAllocator> >
{
  static const char* value()
  {
    return "utils/environmental";
  }

  static const char* value(const ::utils::environmental_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::utils::environmental_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 obstacle_id\n"
"float32 x\n"
"float32 y\n"
;
  }

  static const char* value(const ::utils::environmental_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::utils::environmental_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.obstacle_id);
      stream.next(m.x);
      stream.next(m.y);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct environmental_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::utils::environmental_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::utils::environmental_<ContainerAllocator>& v)
  {
    s << indent << "obstacle_id: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.obstacle_id);
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UTILS_MESSAGE_ENVIRONMENTAL_H
