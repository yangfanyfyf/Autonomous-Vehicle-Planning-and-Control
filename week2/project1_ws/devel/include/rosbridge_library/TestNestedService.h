// Generated by gencpp from file rosbridge_library/TestNestedService.msg
// DO NOT EDIT!


#ifndef ROSBRIDGE_LIBRARY_MESSAGE_TESTNESTEDSERVICE_H
#define ROSBRIDGE_LIBRARY_MESSAGE_TESTNESTEDSERVICE_H

#include <ros/service_traits.h>


#include <rosbridge_library/TestNestedServiceRequest.h>
#include <rosbridge_library/TestNestedServiceResponse.h>


namespace rosbridge_library
{

struct TestNestedService
{

typedef TestNestedServiceRequest Request;
typedef TestNestedServiceResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct TestNestedService
} // namespace rosbridge_library


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::rosbridge_library::TestNestedService > {
  static const char* value()
  {
    return "063d2b71e58b5225a457d4ee09dab6f6";
  }

  static const char* value(const ::rosbridge_library::TestNestedService&) { return value(); }
};

template<>
struct DataType< ::rosbridge_library::TestNestedService > {
  static const char* value()
  {
    return "rosbridge_library/TestNestedService";
  }

  static const char* value(const ::rosbridge_library::TestNestedService&) { return value(); }
};


// service_traits::MD5Sum< ::rosbridge_library::TestNestedServiceRequest> should match
// service_traits::MD5Sum< ::rosbridge_library::TestNestedService >
template<>
struct MD5Sum< ::rosbridge_library::TestNestedServiceRequest>
{
  static const char* value()
  {
    return MD5Sum< ::rosbridge_library::TestNestedService >::value();
  }
  static const char* value(const ::rosbridge_library::TestNestedServiceRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::rosbridge_library::TestNestedServiceRequest> should match
// service_traits::DataType< ::rosbridge_library::TestNestedService >
template<>
struct DataType< ::rosbridge_library::TestNestedServiceRequest>
{
  static const char* value()
  {
    return DataType< ::rosbridge_library::TestNestedService >::value();
  }
  static const char* value(const ::rosbridge_library::TestNestedServiceRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::rosbridge_library::TestNestedServiceResponse> should match
// service_traits::MD5Sum< ::rosbridge_library::TestNestedService >
template<>
struct MD5Sum< ::rosbridge_library::TestNestedServiceResponse>
{
  static const char* value()
  {
    return MD5Sum< ::rosbridge_library::TestNestedService >::value();
  }
  static const char* value(const ::rosbridge_library::TestNestedServiceResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::rosbridge_library::TestNestedServiceResponse> should match
// service_traits::DataType< ::rosbridge_library::TestNestedService >
template<>
struct DataType< ::rosbridge_library::TestNestedServiceResponse>
{
  static const char* value()
  {
    return DataType< ::rosbridge_library::TestNestedService >::value();
  }
  static const char* value(const ::rosbridge_library::TestNestedServiceResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ROSBRIDGE_LIBRARY_MESSAGE_TESTNESTEDSERVICE_H