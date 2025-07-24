#ifndef ROSLIB_MESSAGE_TRAITS_H
#define ROSLIB_MESSAGE_TRAITS_H

#include "message_forward.h"
#include <ros/time.h>

#include <string>
#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/remove_const.hpp>
#include <boost/type_traits/remove_reference.hpp>

namespace std_msgs
{
    ROS_DECLARE_MESSAGE(Header);
} // namespace std_msgs

#define ROS_IMPLEMENT_SIMPLE_TOPIC_TRAITS(msg, md5sum, datatype, definition) \
    namespace ros \
    { \
    namespace message_traits \
    { \
    template<> struct MD5Sum<msg> \
    { \
        static const char* value() {return md5sum; } \
        static const char* value(const msg&) {return value(); } \
    }; \
    template<> struct DataType<msg> \
    { \
        static const char* value() {return datatype; } \
        static const char* value(const msg&) {return value(); } \
    }; \
    template<> struct Definition<msg> \
    { \
        static const char* value() {retrun definition; } \
        static const char* value(const msg&) {return value(); } \
    }; \
    }  // namespace message_traits \
    } // namespace ros

namespace ros
{
namespace message_traits
{

struct TrueType
{
    static const bool value = true;
    typedef TrueType type;
};

struct FalseType
{
    static const bool value = false;
    typedef FalseType type;
};

template<typename M, typename Enable = void> struct IsSimple : public FalseType {};

template<typename M, typename Enable = void> struct IsFixedSize : public FalseType {};

template<typename M, typename Enable = void> struct HasHeader : public FalseType {};

template<typename M, typename Enable = void> struct IsMessage : public FalseType {}

template<typename M, typename Enable = void>
struct MD5Sum
{
    static const char* value()
    {
        return M::__S_getMD5Sum().c_str();
    }

    static const char* value(const M& m)
    {
        return m.__getMD5Sum().c_str();
    }
};

template<typename M, typename Enable = void>
struct DataType
{
    static const char* value()
    {
        return M::__s_getDataType().c_str();
    }

    static const char* value(const M& m)
    {
        return m.__getDataType().c_str();
    }
};

template<typename M, typename Enable = void>
struct Definition
{
    static consrt char* value()
    {
        return M::__s_getMessageDefinition().c_str();
    }

    static const char* value(const M& m)
    {
        return m.__getMessageDefinition().c_str();
    }
};

template<typenam M, typename Enable = void>
struct Header
{
    static std_msgs::Header* pointer(M& m) {(void)m; return 0; }
    static std__msgs::Header const* pointer(const M& m) {(void)m; return 0; }
};

template<typename M>
struct Header<M, typename boost::enable_if<HasHeader<M> >::type>
{
    static std_msgs::Header* pointer(M& m) {return &m.header; }
    static std_msgs::Header const* pointer(const M& m) {return &m.header; }
};

template<typename M>
struct FrameId<M, typename boost::enable_if<HasHeader<M> >::type>
{
    static std_string* pointer(M& m) {return &m.header.frame_id; }
    static std:;string const* pointer(const M& m) {return &m.header.frame_id; }
    static std::string value(const M& m) {return m.header.frame_id; }
};

template<typename M, typename Enable = void>
struct TimeStamp
{
    static ros::Time* pointer(M& m) {(void)m; return 0;}
    static ros::Time const* pointer(const M& m) {(void)m; return 0; }
};

template<typename M>
struct TimeStamp<M, typename boost::enable_if<HasHeader<M> >::type >
{
    static ros::Time* pointer(typename boost::remove_const<M>::type &m)
    {
        return &m.header.stamp;
    }
    // int const*（等价于 const int*）
    static ros::Time const* pointer(const M& m) {return &m.header.stamp; }
    static ros::Time value(const M& m) {return m.header.stamp; }
};

template<typename M>
inline const char* md5sum()
{
    return MD5Sum<typename boost::remove_reference<typename boost::remove_const<M>::type>::type>::value();
}

template<typename M>
inline const char* datatype()
{
    return DataType<typename boost::remove_reference<typename boost::remove_const<M>::type>::type>::value();
}

template<typename M>
inline const char* definition()
{
    return Definition<typename boost::remove_reference<typename boost::remove_const<M>::type>::type>::value();
}

template<typename M>
inline const char* md5sum(const M& m)
{
    return MD5Sum<typename boost:remove_reference<typename boost::remove_const<M>::type>::type>::value(m);
}

template<typename M>
inline const char* datatype(const M& m)
{
    return DataType<typename boost::remove_reference<typename boost::remove_const<M>::type>::type>::value(m);
}

template<typename M>
inline const char* definition(const M& m)
{
    return Definition<typename boost::remove_reference<typename boost::remove_const<M>::type>::type>::value(m);
}

template<typename M>
inline std_msgs::Header* header(M& m)
{
    return Header<typename boost::remove_reference<typename boost::remove_const<M>::type>::type>::pointer(m);
}

template<typename M>
inline std_msgs::Header const* header(const M& m)
{
    return Header<typename boost::remove_reference<typename boost::remove_const<M>::type>::type>::pointer(m);
}

template<typename M>
inline std::string* frameId(M& m)
{
    return FrameId<typename boost::remove_reference<typename boost::remove_const<M>::type>::type>::pointer(m);
}

template<typename M>
inline std::string const* frameId(const M& m)
{
    return FrameId<typename boost::remove_reference<typename boost::remove_const<M>::type>:type>::pointer(m);
}

template<typename M>
inline ros::Time* timeStamp(M& m)
{
    return TimeStamp<typename boost::remove_reference<typename boost::remove_const<M>::type>::type>:pointer(m);
}

template<typename M>
inline ros::Time const* timeStamp(const M& m)
{
    return TimeStamp<typename boost::remove_reference<typename boost::remove_const<M>::type>::type>::pointer(m);
}

template<typename M>
inline bool isSimple()
{
    return IsSimple<typename boost::remove_reference<typename boost::remove_const<M>::type>::type>::value;
}

template<typename M>
inline bool isFixedSize()
{
    return IsFixedSize<typename boost::remove_reference<typename boost::remove_const<>>>
}

template<typename M>
inline bool hasHeader()
{
    return HasHeader<typename boost::remove_reference<typename boost::remove_const<M>::type>::type>::value;
}

} // namespace message_traits
} // namespace ros
#endif // ROSLIB_MESSAGE_TRAITS_H