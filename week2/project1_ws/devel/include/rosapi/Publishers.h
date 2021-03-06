// Generated by gencpp from file rosapi/Publishers.msg
// DO NOT EDIT!


#ifndef ROSAPI_MESSAGE_PUBLISHERS_H
#define ROSAPI_MESSAGE_PUBLISHERS_H

#include <ros/service_traits.h>


#include <rosapi/PublishersRequest.h>
#include <rosapi/PublishersResponse.h>


namespace rosapi
{

struct Publishers
{

typedef PublishersRequest Request;
typedef PublishersResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct Publishers
} // namespace rosapi


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::rosapi::Publishers > {
  static const char* value()
  {
    return "cb37f09944e7ba1fc08ee38f7a94291d";
  }

  static const char* value(const ::rosapi::Publishers&) { return value(); }
};

template<>
struct DataType< ::rosapi::Publishers > {
  static const char* value()
  {
    return "rosapi/Publishers";
  }

  static const char* value(const ::rosapi::Publishers&) { return value(); }
};


// service_traits::MD5Sum< ::rosapi::PublishersRequest> should match
// service_traits::MD5Sum< ::rosapi::Publishers >
template<>
struct MD5Sum< ::rosapi::PublishersRequest>
{
  static const char* value()
  {
    return MD5Sum< ::rosapi::Publishers >::value();
  }
  static const char* value(const ::rosapi::PublishersRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::rosapi::PublishersRequest> should match
// service_traits::DataType< ::rosapi::Publishers >
template<>
struct DataType< ::rosapi::PublishersRequest>
{
  static const char* value()
  {
    return DataType< ::rosapi::Publishers >::value();
  }
  static const char* value(const ::rosapi::PublishersRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::rosapi::PublishersResponse> should match
// service_traits::MD5Sum< ::rosapi::Publishers >
template<>
struct MD5Sum< ::rosapi::PublishersResponse>
{
  static const char* value()
  {
    return MD5Sum< ::rosapi::Publishers >::value();
  }
  static const char* value(const ::rosapi::PublishersResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::rosapi::PublishersResponse> should match
// service_traits::DataType< ::rosapi::Publishers >
template<>
struct DataType< ::rosapi::PublishersResponse>
{
  static const char* value()
  {
    return DataType< ::rosapi::Publishers >::value();
  }
  static const char* value(const ::rosapi::PublishersResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ROSAPI_MESSAGE_PUBLISHERS_H
