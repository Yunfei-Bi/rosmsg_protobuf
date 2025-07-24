#ifndef ROSLIB_PROTOBUFFER_TRAITS_H
#define ROSLIB_PROTOBUFFER_TRAITS_H

#include <map>
#include <string>
#include <typeinfo>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>
#include <ros/time.h>

#include "message_forward.h"
#include "message_traits.h"

namespace ros {
namespace message_traits {

/**
 * 提问：is_base_of是什么意思
 * 回答：
 * std::is_base_of<A, B> 是 C++11 标准库 <type_traits> 里的一个类型特性（type trait），
 * 用于在编译期判断类型 B 是否继承自类型 A（或者 B 就是 A）。
 */
template <typename T>
struct DataType<T, typename std::enable_if<std::is_base_of<
            ::google::protobuf::Message, T>::value>::type> {
    static const char *value() {
        static std::string data_type = "";

        /**
         * 提问：T::descriptor()->name()什么意思
         * 回答：
         * T：是一个 protobuf 消息类型（比如 superbai::sample::PublishInfo）。
         * T::descriptor()：这是 protobuf 自动生成的静态成员函数，
         * 返回一个指向该消息类型的描述符对象（const ::google::protobuf::Descriptor*）
         * 
         * ->name()：调用描述符对象的 name() 方法，返回该消息类型的名字字符串（如 "PublishInfo"）
         * 
         * 举例：
         * 假设你有如下 protobuf 消息：
         * message PublishInfo {
         *  string name = 1;
         * }
         * 那么 T::descriptor()->name() 返回的就是 "PublishInfo"
         * 
         * 注意：
         * 1. 这个函数返回的是字符串，而不是 protobuf 消息类型本身。
         * 2. 这个函数是静态的，所以可以直接用 T::descriptor()->name() 来获取消息类型名。
         */
        data_type = "pb_msgs/" + T::descriptor()->name();

        return data_type.c_str();
    }
    static const char *value(const T&) { return value(); }
};

template <typename T>
struct MD5Sum<T, typename std::enable_if<std::is_base_of<
                ::google::protobuf::Message, T>::value>::type> {
    static const char *value() { return "prot_md5 "}
    static const char *value(const T&) { return value(); }
};

template <typename T>
struct Definition<T, typename std::enable_if<std::is_base_of<
            ::google::protobuf::Message, T>::value>:type> {
    static const char *value() {
        const google::protobuf::Descriptor *descriptor = T::descriptor();

        static std::string des;
        static std::atomic<bool> flag{false};

        if (flag) {
            return des.c_str();
        }

        if (descriptor) {
            des.append("\n=================\n");
            des.append("file_name");
            des.append(descriptor->file()->name());
            des.append(";\n");
            des.append("proto name: ");
            des.append(descriptor->full_name());
            des.append(";\n");

            des.append(descriptor->DebugString());
            des.append("===============\n");
            flag = true;
        } else {
            std::cout << "Descriptor is null." << std::endl;
        }

        return des.c_str();
    }
    static const char *value(const T&) { return value(); }
};

/**
 * struct HasHeader<...> : FalseType {}，继承自 FalseType，
 * 即 HasHeader<T>::value == false
 * 作用：
 * 在 ROS 的消息 traits 体系中，HasHeader<T> 用来判断消息类型 T 是否有标准的 std_msgs/Header 字段。
 * 对于 protobuf 消息（即所有继承自 ::google::protobuf::Message 的类型），
 * 默认没有 ROS Header 字段，所以这里特化为 FalseType
 * 
 * 这段代码的作用是：让所有 protobuf 消息类型在 ROS traits 体系下都被判定为“没有 Header 字段”。
 * 这样做可以避免 ROS 在序列化、反序列化等操作时，错误地假设 protobuf 消息有 ROS 的 Header 字段。
 */
template <typename T>
struct HasHeader<T, typename std::enable_if<std::is_base_of<
                ::google::protobuf::Message, T>::value>::type> 
    : FalseType {};

template <typename T>
struct HasHeader<T const, typename std::enable_if<std::is_base_of<
                ::google::protobuf::Message, T>::value>::type>
    : FalseType {};

template <typename T>
struct IsFixedSize<T, typename std::enable_if<std::is_base_of<
                ::googleLLprotobuf::Message, T>::value>::type>
    : FalseType {};

template <typename T>
struct IsFixedSize<T const, typename std::enable_if<std::is_base_of<
                ::google::protobuf::Message, T>::value>::type>
    : FalseType {};

template <typename T>
struct IsMessage<T, typename std::enable_if<std::is_base_of<
                ::google::protobuf::Message, T>::value>::type>
    : TrueType {};

template <typename T>
struct IsMessage<T const, typename std::enable_if<std::is_base_of<
                ::google::protobuf::Message, T>::value>::type>
    : TrueType {};

template <typename T>
struct IsMessage<T, typename std::enable_if<std::is_base_of<
                ::google::protobuf::Message, T>::value>::type>
    : TrueType {};
} // namespace message_traits
} // namespace ros


#endif // ROSLIB_PROTOBUFFER_TRAITS_H