// Generated by gencpp from file utils/localisation.msg
// DO NOT EDIT!


#ifndef UTILS_MESSAGE_LOCALISATION_H
#define UTILS_MESSAGE_LOCALISATION_H


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
struct localisation_
{
  typedef localisation_<ContainerAllocator> Type;

  localisation_()
    : timestamp(0.0)
    , posA(0.0)
    , posB(0.0)
    , rotA(0.0)
    , rotB(0.0)  {
    }
  localisation_(const ContainerAllocator& _alloc)
    : timestamp(0.0)
    , posA(0.0)
    , posB(0.0)
    , rotA(0.0)
    , rotB(0.0)  {
  (void)_alloc;
    }



   typedef double _timestamp_type;
  _timestamp_type timestamp;

   typedef float _posA_type;
  _posA_type posA;

   typedef float _posB_type;
  _posB_type posB;

   typedef float _rotA_type;
  _rotA_type rotA;

   typedef float _rotB_type;
  _rotB_type rotB;





  typedef boost::shared_ptr< ::utils::localisation_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::utils::localisation_<ContainerAllocator> const> ConstPtr;

}; // struct localisation_

typedef ::utils::localisation_<std::allocator<void> > localisation;

typedef boost::shared_ptr< ::utils::localisation > localisationPtr;
typedef boost::shared_ptr< ::utils::localisation const> localisationConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::utils::localisation_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::utils::localisation_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::utils::localisation_<ContainerAllocator1> & lhs, const ::utils::localisation_<ContainerAllocator2> & rhs)
{
  return lhs.timestamp == rhs.timestamp &&
    lhs.posA == rhs.posA &&
    lhs.posB == rhs.posB &&
    lhs.rotA == rhs.rotA &&
    lhs.rotB == rhs.rotB;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::utils::localisation_<ContainerAllocator1> & lhs, const ::utils::localisation_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace utils

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::utils::localisation_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::utils::localisation_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::utils::localisation_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::utils::localisation_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::utils::localisation_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::utils::localisation_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::utils::localisation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "32d060122e3076e51189db3d2135636c";
  }

  static const char* value(const ::utils::localisation_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x32d060122e3076e5ULL;
  static const uint64_t static_value2 = 0x1189db3d2135636cULL;
};

template<class ContainerAllocator>
struct DataType< ::utils::localisation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "utils/localisation";
  }

  static const char* value(const ::utils::localisation_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::utils::localisation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 timestamp\n"
"float32 posA\n"
"float32 posB\n"
"float32 rotA\n"
"float32 rotB\n"
;
  }

  static const char* value(const ::utils::localisation_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::utils::localisation_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.timestamp);
      stream.next(m.posA);
      stream.next(m.posB);
      stream.next(m.rotA);
      stream.next(m.rotB);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct localisation_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::utils::localisation_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::utils::localisation_<ContainerAllocator>& v)
  {
    s << indent << "timestamp: ";
    Printer<double>::stream(s, indent + "  ", v.timestamp);
    s << indent << "posA: ";
    Printer<float>::stream(s, indent + "  ", v.posA);
    s << indent << "posB: ";
    Printer<float>::stream(s, indent + "  ", v.posB);
    s << indent << "rotA: ";
    Printer<float>::stream(s, indent + "  ", v.rotA);
    s << indent << "rotB: ";
    Printer<float>::stream(s, indent + "  ", v.rotB);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UTILS_MESSAGE_LOCALISATION_H