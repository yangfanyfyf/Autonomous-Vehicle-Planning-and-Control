// Generated by gencpp from file rosapi/ServiceResponseDetails.msg
// DO NOT EDIT!


#ifndef ROSAPI_MESSAGE_SERVICERESPONSEDETAILS_H
#define ROSAPI_MESSAGE_SERVICERESPONSEDETAILS_H

#include <ros/service_traits.h>


#include <rosapi/ServiceResponseDetailsRequest.h>
#include <rosapi/ServiceResponseDetailsResponse.h>


namespace rosapi
{

struct ServiceResponseDetails
{

typedef ServiceResponseDetailsRequest Request;
typedef ServiceResponseDetailsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ServiceResponseDetails
} // namespace rosapi


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::rosapi::ServiceResponseDetails > {
  static const char* value()
  {
    return "bdbf5d5ad601e3c2244ad2f8692bd791";
  }

  static const char* value(const ::rosapi::ServiceResponseDetails&) { return value(); }
};

template<>
struct DataType< ::rosapi::ServiceResponseDetails > {
  static const char* value()
  {
    return "rosapi/ServiceResponseDetails";
  }

  static const char* value(const ::rosapi::ServiceResponseDetails&) { return value(); }
};


// service_traits::MD5Sum< ::rosapi::ServiceResponseDetailsRequest> should match
// service_traits::MD5Sum< ::rosapi::ServiceResponseDetails >
template<>
struct MD5Sum< ::rosapi::ServiceResponseDetailsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::rosapi::ServiceResponseDetails >::value();
  }
  static const char* value(const ::rosapi::ServiceResponseDetailsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::rosapi::ServiceResponseDetailsRequest> should match
// service_traits::DataType< ::rosapi::ServiceResponseDetails >
template<>
struct DataType< ::rosapi::ServiceResponseDetailsRequest>
{
  static const char* value()
  {
    return DataType< ::rosapi::ServiceResponseDetails >::value();
  }
  static const char* value(const ::rosapi::ServiceResponseDetailsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::rosapi::ServiceResponseDetailsResponse> should match
// service_traits::MD5Sum< ::rosapi::ServiceResponseDetails >
template<>
struct MD5Sum< ::rosapi::ServiceResponseDetailsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::rosapi::ServiceResponseDetails >::value();
  }
  static const char* value(const ::rosapi::ServiceResponseDetailsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::rosapi::ServiceResponseDetailsResponse> should match
// service_traits::DataType< ::rosapi::ServiceResponseDetails >
template<>
struct DataType< ::rosapi::ServiceResponseDetailsResponse>
{
  static const char* value()
  {
    return DataType< ::rosapi::ServiceResponseDetails >::value();
  }
  static const char* value(const ::rosapi::ServiceResponseDetailsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ROSAPI_MESSAGE_SERVICERESPONSEDETAILS_H
