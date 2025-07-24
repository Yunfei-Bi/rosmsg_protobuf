#ifndef ROSLIB_MESSAGE_OPERATIONS_H
#define ROSLIB_MESSAGE_OPERATIONS_H

#include <ostream>

namespace ros {

namespace message_operations {

/**
 * 这是一个模板结构体，针对任意类型 M 都可以实例化一个 Printer。
 * Stream 可以是任何流类型（如 std::ostream）
 * s 是输出流对象
 * indent 是缩进字符串（用于格式化输出，虽然这里没用到）
 * value 是要输出的值，类型为 M
 * 
 * s << value << "\n";
 * 把 value 输出到流 s，并换行。
 * 要求 T 类型支持 operator<<（即能直接用流输出）。
 */
template <typename M>
struct Printer {
    template<typename Stream>
    static void stream(Stream& s, const std::string& indent, const M& value) {
        s << value << "\n";
    }
};

/**
 * 这里单独拿int8_t和uint8_t出来的原因是
 * 他们的定义如下：
 * typedef signed char __int8_t;
 * typedef unsigned char __uint8_t;
 * 在 C++ 中， int8_t 和  uint8_t 通常被定义为 signed char 和 unsigned char。
 * 当你使用 operator<< 输出它们时，流会将它们当作字符而不是数字来处理。
 * 
 * 这里的特化是为了确保 int8_t 作为数字输出，而不是字符。
 */
template <>
struct Printer<int8_t> {
    template <typename Stream>
    static void stream(Stream& s, const std::string& indent, int8_t value) {

        // 输出数字而不是字符
        s << static_cast<int32_t>(value) << "\n";
    }
};

/**
 * 这里的特化是为了确保 uint8_t 作为数字输出，而不是字符。
 */
template <>
struct Printer<uint8_t> {
    template <typename Stream>
    static void stream(Stream& s, const std::string& indent, uint8_t value) {
        
        // 输出数字而不是字符
        s << static_cast<uint32_t>(value) << "\n";
    }
};


} // message_operations

} // namespace ros


#endif // ROSLIB_MESSAGE_OPERATIONS_H