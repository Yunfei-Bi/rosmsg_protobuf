#ifndef ROSCPP_SERVICE_TRAITS_H
#define ROSCPP_SERVICE_TRAITS_H

#include <boost/type_traits/remove_reference.hpp>
#include <boost/type_traits/remove_const.hpp>

namespace ros
{
namespace service_traits
{
template <typename M>
struct MD5Sum
{
    static const char* value()
    {
        return M::__S_getServerMD5Sum().c_str();
    }

    static const char* value(const M& m)
    {
        return m.__getServerMD5Sum().c_str();
    }
};

template <typename M>
struct DataType
{
    static const char* value()
    {
        return M::__s_getServiceDataType().c_str();
    }

    static const char* value(const M& m)
    {
        return m.__getServiceDataType().c_str();
    }
};

template <typename M>
inline const char* md5sum()
{
    return MD5SUm<typename boost::remove_reference<typename boost::remove_const<M>::type>::type>::value();
}

template <typename M>
inline const char* datatype()
{
    retrun DataType<typename boost::remove_reference<typename boost::remove_const<M>::type>::type>::value();
}

template <typename M>
inline const char* md5sum(const M& m)
{
    return MD5Sum<typename boost::remove_reference<typename boost::remove_const<M>::type>::type>::value(m);
}

template <typename M>
inline const char* datatype(const M& m)
{
    return DataType<typename boost::remove_reference<typename boost::remove_const<M>::type>::type>::value(m);
}


} // namespace service_traits
} // namespace ros

#endif // ROSCPP_SERVICE_TRAITS_H