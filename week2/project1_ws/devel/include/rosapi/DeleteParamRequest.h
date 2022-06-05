// Generated by gencpp from file rosapi/DeleteParamRequest.msg
// DO NOT EDIT!


#ifndef ROSAPI_MESSAGE_DELETEPARAMREQUEST_H
#define ROSAPI_MESSAGE_DELETEPARAMREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rosapi
{
template <class ContainerAllocator>
struct DeleteParamRequest_
{
  typedef DeleteParamRequest_<ContainerAllocator> Type;

  DeleteParamRequest_()
    : name()  {
    }
  DeleteParamRequest_(const ContainerAllocator& _alloc)
    : name(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  _name_type name;





  typedef boost::shared_ptr< ::rosapi::DeleteParamRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rosapi::DeleteParamRequest_<ContainerAllocator> const> ConstPtr;

}; // struct DeleteParamRequest_

typedef ::rosapi::DeleteParamRequest_<std::allocator<void> > DeleteParamRequest;

typedef boost::shared_ptr< ::rosapi::DeleteParamRequest > DeleteParamRequestPtr;
typedef boost::shared_ptr< ::rosapi::DeleteParamRequest const> DeleteParamRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rosapi::DeleteParamRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rosapi::DeleteParamRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::rosapi::DeleteParamRequest_<ContainerAllocator1> & lhs, const ::rosapi::DeleteParamRequest_<ContainerAllocator2> & rhs)
{
  return lhs.name == rhs.name;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::rosapi::DeleteParamRequest_<ContainerAllocator1> & lhs, const ::rosapi::DeleteParamRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace rosapi

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::rosapi::DeleteParamRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rosapi::DeleteParamRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rosapi::DeleteParamRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rosapi::DeleteParamRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rosapi::DeleteParamRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rosapi::DeleteParamRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rosapi::DeleteParamRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c1f3d28f1b044c871e6eff2e9fc3c667";
  }

  static const char* value(const ::rosapi::DeleteParamRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc1f3d28f1b044c87ULL;
  static const uint64_t static_value2 = 0x1e6eff2e9fc3c667ULL;
};

template<class ContainerAllocator>
struct DataType< ::rosapi::DeleteParamRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rosapi/DeleteParamRequest";
  }

  static const char* value(const ::rosapi::DeleteParamRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rosapi::DeleteParamRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string name\n"
;
  }

  static const char* value(const ::rosapi::DeleteParamRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rosapi::DeleteParamRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.name);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DeleteParamRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rosapi::DeleteParamRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rosapi::DeleteParamRequest_<ContainerAllocator>& v)
  {
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSAPI_MESSAGE_DELETEPARAMREQUEST_H