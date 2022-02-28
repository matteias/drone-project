// Generated by gencpp from file crazyflie_gazebo/RemoveCrazyflie.msg
// DO NOT EDIT!


#ifndef CRAZYFLIE_GAZEBO_MESSAGE_REMOVECRAZYFLIE_H
#define CRAZYFLIE_GAZEBO_MESSAGE_REMOVECRAZYFLIE_H

#include <ros/service_traits.h>


#include <crazyflie_gazebo/RemoveCrazyflieRequest.h>
#include <crazyflie_gazebo/RemoveCrazyflieResponse.h>


namespace crazyflie_gazebo
{

struct RemoveCrazyflie
{

typedef RemoveCrazyflieRequest Request;
typedef RemoveCrazyflieResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct RemoveCrazyflie
} // namespace crazyflie_gazebo


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::crazyflie_gazebo::RemoveCrazyflie > {
  static const char* value()
  {
    return "636fe5e07550f026d28388e95c38a9f4";
  }

  static const char* value(const ::crazyflie_gazebo::RemoveCrazyflie&) { return value(); }
};

template<>
struct DataType< ::crazyflie_gazebo::RemoveCrazyflie > {
  static const char* value()
  {
    return "crazyflie_gazebo/RemoveCrazyflie";
  }

  static const char* value(const ::crazyflie_gazebo::RemoveCrazyflie&) { return value(); }
};


// service_traits::MD5Sum< ::crazyflie_gazebo::RemoveCrazyflieRequest> should match
// service_traits::MD5Sum< ::crazyflie_gazebo::RemoveCrazyflie >
template<>
struct MD5Sum< ::crazyflie_gazebo::RemoveCrazyflieRequest>
{
  static const char* value()
  {
    return MD5Sum< ::crazyflie_gazebo::RemoveCrazyflie >::value();
  }
  static const char* value(const ::crazyflie_gazebo::RemoveCrazyflieRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::crazyflie_gazebo::RemoveCrazyflieRequest> should match
// service_traits::DataType< ::crazyflie_gazebo::RemoveCrazyflie >
template<>
struct DataType< ::crazyflie_gazebo::RemoveCrazyflieRequest>
{
  static const char* value()
  {
    return DataType< ::crazyflie_gazebo::RemoveCrazyflie >::value();
  }
  static const char* value(const ::crazyflie_gazebo::RemoveCrazyflieRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::crazyflie_gazebo::RemoveCrazyflieResponse> should match
// service_traits::MD5Sum< ::crazyflie_gazebo::RemoveCrazyflie >
template<>
struct MD5Sum< ::crazyflie_gazebo::RemoveCrazyflieResponse>
{
  static const char* value()
  {
    return MD5Sum< ::crazyflie_gazebo::RemoveCrazyflie >::value();
  }
  static const char* value(const ::crazyflie_gazebo::RemoveCrazyflieResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::crazyflie_gazebo::RemoveCrazyflieResponse> should match
// service_traits::DataType< ::crazyflie_gazebo::RemoveCrazyflie >
template<>
struct DataType< ::crazyflie_gazebo::RemoveCrazyflieResponse>
{
  static const char* value()
  {
    return DataType< ::crazyflie_gazebo::RemoveCrazyflie >::value();
  }
  static const char* value(const ::crazyflie_gazebo::RemoveCrazyflieResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // CRAZYFLIE_GAZEBO_MESSAGE_REMOVECRAZYFLIE_H