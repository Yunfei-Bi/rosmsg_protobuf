#ifndef ROSLIB_MESSAGE_FORWARD_H
#define ROSLIB_MESSAGE_FORWARD_H

#include <cstddef>

#ifndef _LIBCPP_VERSION
namespace std
{
template<typename T> class shared_ptr;
}
#else
#include <memory>
#endif

#define ROS_DECLARE_MESSAGE_WITH_ALLOCATOR(msg, new_name, alloc) \
    template<class Allocator> struct msg##_; \
    typedef msg##_<alloc<void> > new name; \
    typedef boost::shared_ptr<new_name> new_name##Ptr; \
    typedef boost::shared_ptr<new_name const> new_name##ConstPtr;

#define ROS_DECLARE_MESSAGE(msg) ROS_DECLARE_MESSAGE_WITH_ALLOCATOR(msg, msg, std::allocator)

#endif // ROSLIB_MESSAGE_FORWARD_H
