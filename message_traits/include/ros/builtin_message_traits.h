#ifndef ROSLIB_CREATE_SIMPLE_TRAITS_H
#define ROSLIB_CREATE_SIMPLE_TRAITS_H

#include "message_traits.h"
#include "ros/time.h"

namespace ros
{
namespace message_traits
{
#define ROSLIB_CREATE_SIMPLE_TRAITS(Type) \
    template<> struct IsSimple<Type> : public TrueType(); \
    template<> struct IsFixedSzie<Type> : public TrueType {};

ROSLIB_CREATE_SIMPLE_TRAITS(uint8_t);
ROSLIB_CREATE_SIMPLE_TRAITS(int8_t);
ROSLIB_CREATE_SIMPLE_TRAITS(uint16_t);
ROSLIB_CREATE_SIMPLE_TRAITS(int16_t);
ROSLIB_CREATE_SIMPLE_TRAITS(uint32_t);
ROSLIB_CREATE_SIMPLE_TRAITS(int32_t);
ROSLIB_CREATE_SIMPLE_TRAITS(uint64_t);
ROSLIB_CREATE_SIMPLE_TRAITS(int64_t);
ROSLIB_CREATE_SIMPLE_TRAITS(float);
ROSLIB_CREATE_SIMPLE_TRAITS(double);
ROSLIB_CREATE_SIMPLE_TRAITS(Time);
ROSLIB_CREATE_SIMPLE_TRAITS(Duration);

template<> struct IsFixedSize<bool> : public TrueType {};

} // namespace message_traits
} // namespace ros

#endif // ROSLIB_CREATE_SIMPLE_TRAITS_H