#ifndef ROSCPP_SEREIALIZATION_PROTOBUF_H
#define ROSCPP_SEREIALIZATION_PROTOBUF_H

#include <google/protobuf/message.h>

#include "serialization.h"

namespace ros {

namespace serialization {

/**
 * protobuffer serialization
 */
template <typename T>
struct Serializer<T, typename std::enable_if<std::is_base_of<
                    ::google::protobuf::Message, T>::value>::type> {
    
    template <typename Stream>
    inline static void write(Stream &stream, const T &t) {
        std::string pb_str;
        t.SerializeToString(&pb_str);

        // 4个字节
        uint32_t len = (uint32_t)pb_str.size();
        stream.next(len);

        if (len > 0) {
            memcpy(stream.advance((unint32_t)len), pb_str.data(), len);
        }
    }

    /**
     * ros反序列化接口
     */
    template <typename Stream>
    inline static void read(Stream &stream, T &t) {
        uint32_t len;
        // IStream
        stream.next(len);

        std::string pb_str;
        if (len > 0) {
            const char *data_ptr = 
                reinterpret_cast<const char *>(stream.advance(len));
            pb_str = std::string(data_ptr, len);
        } else {
            pb_str.clear();
        }
        t.ParseFromString(pb_str);
    }

    inline static uint_32_t serializedLength(const T &t) {
        std::string pb_str;
        t.SerializedToString(&pb_str);

        return 4 + (uint32_t)pb_str.size();
    }
};

} // namespace serialization
} // namespace ros




#endif // ROSCPP_SEREIALIZATION_PROTOBUF_H