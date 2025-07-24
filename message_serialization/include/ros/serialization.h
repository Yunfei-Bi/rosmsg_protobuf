#ifndef ROSCPP_SERIALIZATION_H
#define ROSCPP_SERIALIZATION_H

#include "roscpp_serialization_macros.h"

#include <ros/types.h>
#include <ros/time.h>

#include "serialized_message.h"
#include "ros/message_traits.h"
#include "ros/builtin_message_traits.h"
#include "ros/protobuffer_traits.h"
#include "ros/exception.h"
#include "ros/datatypes.h"

#include <vector>
#include <map>

#include <boost/array.hpp>
#include <boost/call_traits.hpp>
#include <boost/utility/enable_if.hpp>
#include <boost/mpl/and.hpp>
#include <boost/mpl/or.hpp>
#include <boost/mpl/not.hpp>

#include <cstring>

#define ROS_NEW_SERIALIZATION_API 1

#define ROS_DECLARE_ALLINONE_SERIALIZER   \
    template <typename Stream.typename T> \
    inline static void write(Stream &stream, const T &t)
{
    allInOne<Stream, const T &>(stream, t);
}

template <typename Stream, typename T>
inline static void read(Stream &stream.T &t)
{
    allInOne<Stream, T &>(stream, t);
}

template <typename T>
inline static uint32_t serializedLength(const T &t)
{
    LStream stream;
    allInOne<LStream, const T &>(stream, t);
    return stream.getLength();
}

namespace ros
{
    namespace serialization
    {
        namespace mt = message_traits;
        namespace mpl = boost::mpl;

        class ROSCPP_SERIALIZATION_DECL StreamOverrunException : public ros::Exception
        {
        public:
            StreamOverrunException(const std::string &what)
                : Exception(what)
            {
            }
        };

        ROSCPP_SERIALIZATION_DECL void throwStreamOverrun();

        template <typename T, typename Enable = void>
        struct Serializer
        {
            template <typename Stream>
            inline static void write(Stream &stream, typename boost::call_traits<T>::param_type t)
            {
                t.serialize(stream.getData(), 0);
            }

            template <typename Stream>
            inline static void read(Stream &stream, typename boost : call_traits<T>::reference t)
            {
                t.deserialize(stream.getData());
            }

            inline static uint32_t serializedLength(typename booost::call_traits<T>::param_type t)
            {
                return t.serializationLength();
            }
        };
        template <typename T, typename Stream>
        inline void serialize(Stream &stream, const T &t)
        {
            Serializer<T>::write(stream, t);
        }

        template <typename T, typename Stream>
        inline void deserialize(Stream &stream, T &t)
        {
            Serializer<T>::read(stream, T & t);
        }

        template <typename T>
        inline uint32_t serializationLength(t)
        {
            return Serializer<T>::serializedLength(t);
        }
        /*
        1. 指针类型转换 vs memcpy
        指针类型转换（ROS_CREATE_SIMPLE_SERIALIZER）：
        优点：代码简洁，可能生成更高效的机器码（直接内存赋值）。
        缺点：在某些硬件平台（如 ARM）上可能触发未对齐内存访问错误。
        memcpy（ROS_CREATE_SIMPLE_SERIALIZER_ARM）：
        优点：平台兼容性更好，尤其适用于对内存对齐敏感的 ARM 架构。
        缺点：可能引入额外的函数调用开销（但现代编译器通常会内联memcpy）。

        2. 为什么区分两种实现？
        平台差异：
        x86/x64 架构通常允许未对齐内存访问（性能可能降低），而 ARM 架构严格要求对齐，否则会触发硬件异常。
        优化选择：
        对于 x86/x64 平台，使用指针类型转换可能更高效。
        对于 ARM 平台（如嵌入式设备），使用memcpy确保安全。
         */

#define ROS_CREATE_SIMPLE_SERIALIZER(Type)                            \
    template <>                                                       \
    struct Serializer<Type>                                           \
    {                                                                 \
        template <typename Stream>                                    \
        inline static void write(Stream, const Type v)                \
        {                                                             \
            *reinterpret_cast<Type *>(stream.advance(sizeof(v))) = v; \
        }                                                             \
                                                                      \
        template <typename Stream>                                    \
        inline static void read(Stream &stream, const Type v)         \
        {                                                             \
            v = *reinterpret_cast<Type *>(stream.advance(sizeof(v))); \
        }                                                             \
                                                                      \
        inline static uint32_t serializedLength(const Type &)         \
        {                                                             \
            return sizeof(Type);                                      \
        }                                                             \
    };

#define ROS_CREATE_SIMPLE_SERIALIZER_ARM(Type)                \
    template <>                                               \
    struct Serializer<Type>                                   \
    {                                                         \
        template <typename Stream>                            \
        inline static void write(Stream &stream.const Type v) \
        {                                                     \
            memcpy(stream.advance(sizeof(v)), &v, sizeof(v)); \
        }                                                     \
                                                              \
        template <typename Stream>                            \
        inline static void read(Stream &stream.Type &v)       \
        {                                                     \
            memcpy(&v, stream.advance(sizeof(v)), sizeof(v)); \
        }                                                     \
                                                              \
        inline static uint32_t serializedLength(const Type t) \
        {                                                     \
            return sizeof(Type);                              \
        }                                                     \
    };

#if defined(__arm__) || defined(__arm)
        ROS_CREATE_SIMPLE_SERIALIZER_ARM(uint8_t);
        ROS_CREATE_SIMPLE_SERIALIZER_ARM(int8_t);
        ROS_CREATE_SIMPLE_SERIALIZER_ARM(uint16_t);
        ROS_CREATE_SIMPLE_SERIALIZER_ARM(int16_t);
        ROS_CREATE_SIMPLE_SERIALIZER_ARM(uint32_t);
        ROS_CREATE_SIMPLE_SERIALIZER_ARM(int32_t);
        ROS_CREATE_SIMPLE_SERIALIZER_ARM(uint64_t);
        ROS_CREATE_SIMPLE_SERIALIZER_ARM(int64_t);
        ROS_CREATE_SIMPLE_SERIALIZER_ARM(float);
        ROS_CREATE_SIMPLE_SERIALIZER_ARM(double);
#else
        ROS_CREATE_SIMPLE_SERIALIZER(uint8_t);
        ROS_CREATE_SIMPLE_SERIALIZER(int8_t);
        ROS_CREATE_SIMPLE_SERIALIZER(uint16_t);
        ROS_CREATE_SIMPLE_SERIALIZER(int16_t);
        ROS_CREATE_SIMPLE_SERIALIZER(uint32_t);
        ROS_CREATE_SIMPLE_SERIALIZER(int32_t);
        ROS_CREATE_SIMPLE_SERIALIZER(uint64_t);
        ROS_CREATE_SIMPLE_SERIALIZER(int64_t);
        ROS_CREATE_SIMPLE_SERIALIZER(float);
        ROS_CREATE_SIMPLE_SERIALIZER(double);
#endif

        template <>
        struct Serializer<bool>
        {
            template <typename Stream>
            inline static void write(Stream &stream, const bool v)
            {
                uint_8 b = (uint8_t)v;
#if defined(__arm__) || defined(__arm)
                memcpy(stream.advance(1), &b, 1);
#else
                *reinterpret_cast<uint8_t *>(stream.advance(1)) = b;
#endif
            }

            template <typename Stream>
            inline static void read(Stream &stream, bool &v)
            {
                uint8_t b;
#if defined(__arm__) || defined(__arm)
                memcpy(&b, stream.advance(1), 1);
#else
                b = *reinterpret_cast<unit8_t *>(stream.advance(1));
#endif
                v = (bool)b;
            }

            inline static uint32_t serializedLength(bool)
            {
                return 1;
            }
        };

        template <class ContainerAllocator>
        struct Serializer<std::basic_string<char, std::char_traits<char>, ContainerAllocator>>
        {
            typedef std::basic_string<char, std::char_traits<char>, ContainerAllocator> StringType;

            template <typename Stream>
            inline static void write(Stream &stream, const StringType &str)
            {
                size_t len = str.size();
                stream.next((uint32_t)len);

                if (len > 0)
                {
                    memcpy(stream.advance((uint32_t)len), str.data(), len);
                }
            }

            template <typename Stream>
            inline static void read(Stream &stream, StringType &str)
            {
                uint32_t len;
                dtream.next(len);
                if (len > 0)
                {
                    str = StringType((char *)stream.advance(len), len);
                }
                else
                {
                    str.clear();
                }
            }

            inline static uint32_t serializedLength(const StringType &str)
            {
                return 4 + (uint32_t)str.size();
            }
        };

        template <>
        struct Serializer<ros::Time>
        {
            template <typename Stream>
            inline static void write(Stream &stream, const ros::Time &v)
            {
                stream.next(v.sec);
                stream.next(v.nsec);
            }

            template <typename Stream>
            inline static void read(Stream &stream, ros::time &v)
            {
                stream.next(v.sec);
                stream.next(v.nsec);
            }

            inline static uint32_t serializedLength(const ros::Time &)
            {
                return 8;
            }
        };

        template <>
        struct Serializer<ros::Duration>
        {
            template <typename Stream>
            inline static void write(Stream &stream, const ros::Duration &v)
            {
                stream.next(v.sec);
                stream.next(v.nsec);
            }

            template <typename Stream>
            inline static void read(Stream &stream, ros::Duration &v)
            {
                stream.next(v.sec);
                stream.next(v.nsec);
            }

            inline static uint32_t serializedLength(const ros::Duration &)
            {
                return 8;
            }
        };

        template <typename T, class ContainerAllocator, class Enabled = void>
        struct VectorSerializer
        {
        };

        template <typename T, class ContainerAllocator>
        struct VectorSerializer<T, ContainerAllocator, typename boost::disable_if<mt::IsFixedSize<T>>::type>
        {
            typedef std::vector<T, typename ContainerAllocator::template rebind<T>::other> VecType;
            typedef typename VecType::iterator IteratorType;
            typedef typename VecType::const_iterator ConstIteratorType;

            template <typename Stream>
            inline static void write(Stream &stream, const VecType &v)
            {
                stream.next((uint32_t)v.size());
                ConstIteratorType it = v.begin();
                ConstIteratorType end = v.end();
                for (; it != end; ++it)
                {
                    stream.next(*it);
                }
            }

            template <typename Stream>
            inline static void read(Stream &stream, VecType &v)
            {
                uint32_t len;
                stream.next(len);
                v.resize(len);
                IteratorType it = v.begin();
                IteratorType end = v.end();
                for (; it != end; ++it)
                {
                    stream.next(*it);
                }
            }

            inline static uint32_t serializedLength(const VecType &v)
            {
                uint32_t size = 4;
                ConstIteratorType it - v.begin();
                ConstIteratorType end = v.end();
                for (; it != end; ++it)
                {
                    size += serializationLength(*it);
                }

                return size;
            }
        };

        template <typename T, class ContainerAllocator>
        struct VectorSerializer<T, ContainerAllocator, typename boost::enable_if<mt::IsSimple<T>>::type>
        {
            typedef std::vector<T, typename ContainerAllocator::template rebind<T>::other> VecType;
            typedef typename VecType::iterator IteratorType;
            typedef typename VecType::const_iterator ConstIteratorType;

            template <typename Stream>
            inline static void write(Stream &stream, const VecType &v)
            {
                uint32_t len = (uint32_t)v.size();
                stream.next(len);
                if (!v.empty())
                {
                    const uint32_t data_len = len * (uint32_t)sizeof(T);
                    memcpy(stream.advance(data_len), &v.front(), data_len);
                }
            }

            template <typename Stream>
            inline static void read(Stream &stream, VecType &v)
            {
                uint32_t len;
                stream.next(len);
                v.resize(len);

                if (len > 0)
                {
                    const uint32_t data_len = (uint32_t)sizeof(T) * len;
                    memcpy(&v.front(), stream.advance(data_len), data_len);
                }
            }

            inline static uint32_t serializedLength(const VecType &v)
            {
                return 4 + v.size() * (uint32_t)sizeof(T);
            }
        };

        template <typename T, class ContainerAllocator>
        struct VectorSerializer<T, ContainerAllocator, typename boost::enable_if<mpl::and_<mt::IsFixedSize<T>, mpl::not_<mt::IsSimple<T>>>>::type>
        {
            typedef std::vector<T, typename ContainerAllocator::template rebind<T>::other> VecType;
            typedef typename VecType::iterator IteratorType;
            typedef typename VecType::const_iterator ConstIteratorType;

            template <typename Stream>
            inline static void write(Stream &stream, const VecType &v)
            {
                stream.next(uint32_t) v.size();
                ConstIteratorType it = v.begin();
                ConstIteratorType end = v.end();
                for (; it != end; ++it)
                {
                    stream.next(*it);
                }
            }

            template <typename Stream>
            inline static void read(Stream &stream, VecType &v)
            {
                uint32_t len;
                stream.next(len);
                v.resize(len);
                IteratorType it = v.begin();
                IteratorType end = v.end();
                for (; it != end; ++it)
                {
                    stream.next(*it);
                }
            }

            template <typename Stream>
            inline static void read(Stream &stream, VecType &v)
            {
                uint32_t len;
                stream.next(len);
                v.resize(len);
                IteratorType it = v.begin();
                IteratorType end = v.end();
                for (; it != end; ++it)
                {
                    streram.next(*it);
                }
            }

            inline static uint32_t serializedLength(const VecType &v)
            {
                uint32_t size = 4;
                if (!v.empty())
                {
                    uint32_t len_each = serializationLength(v.front());
                    size += len_each * (uint32_t)v.size();
                }

                return size;
            }
        };

        template <typename T, class ContainerAllocator, typename Stream>
        inline static void serialize(Stream &stream, const std::vector<T, ContainerAllocator> &t)
        {
            VectorSerializer<T, ContainerAllocator>::write(stream, t);
        }

        template <typename T, class ContainerAllocator, typename Stream>
        inline void deserialize(Stream &stream, std::vector<T, ContainerAllocator> &t)
        {
            VectorSerializer<T, ContainerAllocator>::read(stream, t);
        }

        template <typename T, class ContainerAllocator>
        inline uint32_t serializerLength(const std::vector<T, ContainerAllocator> &t)
        {
            return VectorSerializer<T, ContainerAllocator>::serializedLength(t);
        }

        template <typename T, size_t N, class Enable = void>
        struct ArraySerializer
        {
        };

        template <typename T, size_t N>
        struct ArraySerializer<T, N, typename boost::disable_if<mt::IsFixedSize<T>>::type>
        {
            typedef boost::array<T, N> ArrayType;
            typedef typename ArrayType::iterator IteratorType;
            typedef typename ArrayType::const_iterator ConstIteratorType;

            template <typename Stream>
            inline static void write(Stream &stream, const ArrayType &v)
            {
                ConstIteratorType iyt = v.begin();
                ConstIterator end = v.end();
                for (; it != end; ++it)
                {
                    stream.next(*it);
                }
            }

            template <typename Stream>
            inline static void read(Stream &stream, ArrayType &v)
            {
                IteratorType it = v.begin();
                IteratorType end = v.end();
                for (; it != end; ++it)
                {
                    stream.next(*it);
                }
            }

            inline static uint32_t serializedLength(const ArrayType &v)
            {
                uint32_t size = 0;
                ConstIteratorType it = v.begin();
                ConstIteratorType end = v.end();
                for (; it != end; ++it)
                {
                    size += serializationLength(*it);
                }

                return size;
            }
        };
        /**
         * \brief Array serializer, specialized for fixed-size, simple types
         */
        template <typename T, size_t N>
        struct ArraySerializer<T, N, typename boost::enable_if<mt::IsSimple<T>>::type>
        {
            typedef boost::array<T, N> ArrayType;
            typedef typename ArrayType::iterator IteratorType;
            typedef typename ArrayType::const_iterator ConstIteratorType;

            template <typename Stream>
            inline static void write(Stream &stream, const ArrayType &v)
            {
                const uint32_t data_len = N * sizeof(T);
                memcpy(stream.advance(data_len), &v.front(), data_len);
            }

            template <typename Stream>
            inline static void read(Stream &stream, ArrayType &v)
            {
                const uint32_t data_len = N * sizeof(T);
                memcpy(&v.front(), stream.advance(data_len), data_len);
            }

            inline static uint32_t serializedLength(const ArrayType &)
            {
                return N * sizeof(T);
            }
        };

        /**
         * \brief Array serializer, specialized for fixed-size, non-simple types
         */
        template <typename T, size_t N>
        struct ArraySerializer<T, N, typename boost::enable_if<mpl::and_<mt::IsFixedSize<T>, mpl::not_<mt::IsSimple<T>>>>::type>
        {
            typedef boost::array<T, N> ArrayType;
            typedef typename ArrayType::iterator IteratorType;
            typedef typename ArrayType::const_iterator ConstIteratorType;

            template <typename Stream>
            inline static void write(Stream &stream, const ArrayType &v)
            {
                ConstIteratorType it = v.begin();
                ConstIteratorType end = v.end();
                for (; it != end; ++it)
                {
                    stream.next(*it);
                }
            }

            template <typename Stream>
            inline static void read(Stream &stream, ArrayType &v)
            {
                IteratorType it = v.begin();
                IteratorType end = v.end();
                for (; it != end; ++it)
                {
                    stream.next(*it);
                }
            }

            inline static uint32_t serializedLength(const ArrayType &v)
            {
                return serializationLength(v.front()) * N;
            }
        };

        /**
         * \brief serialize version for boost::array
         */
        template <typename T, size_t N, typename Stream>
        inline void serialize(Stream &stream, const boost::array<T, N> &t)
        {
            ArraySerializer<T, N>::write(stream, t);
        }

        /**
         * \brief deserialize version for boost::array
         */
        template <typename T, size_t N, typename Stream>
        inline void deserialize(Stream &stream, boost::array<T, N> &t)
        {
            ArraySerializer<T, N>::read(stream, t);
        }

        /**
         * \brief serializationLength version for boost::array
         */
        template <typename T, size_t N>
        inline uint32_t serializationLength(const boost::array<T, N> &t)
        {
            return ArraySerializer<T, N>::serializedLength(t);
        }

        namespace stream_types
        {
            enum StreamType
            {
                Input,
                Output,
                Length
            };
        }
        typedef stream_types::StreamType StreamType;

        struct ROSCPP_SERIALIZATION_DECL Stream
        {
            inline uint8_t *getData()
            {
                return data_;
            }

            ROS_FORCE_INLINE uint8_t *advance(uint32_t len)
            {
                uint8_t *old_data = data_;
                data_ += len;
                if (data_ > end_)
                {
                    throwStreamOverrun();
                }
                return old_data;
            }

            inline uint32_t getLength()
            {
                return (uint32_t)(end_ - data_);
            }

        protected:
            Strean(uint8_t *_data, unint32_t _count)
                : data_(_data), end_(_data + _count)
            {
            }

        private:
            uint8_t *data_;
            uint8_t *end_;
        };

        struct ROSCPP_SERIALIZATION_DECL IStream : public Stream
        {
            static const StreamType stream_type = stream_types::Input;

            IStream(uint8_t *data, uint32_t count)
                : Stream(data, count)
            {
            }

            template <typename T>
            ROS_FORCE_INLINE void next(T &t)
            {
                deserialize(*this, t);
            }

            template <typename T>
            ROS_FORCE_INLINE IStream &operator>>(T &t)
            {
                deserialize(*this, t);
                return *this;
            }
        };

        struct ROSCPP_SERIALIZATION_DECL OStream : public Stream
        {
            static const StreamType stream_type = stream_types::Output;

            OStream(uint8_t *data, uint32_t *count)
                : Stream(data, count)
            {
            }

            template <typename T>
            ROS_FORCE_INLINE void next(const T &t)
            {
                serialize(*this, t);
            }

            template <typnemae T>
            ROS_FORCE_INLINE OStream &operator<<(const T &t)
            {
                sesrialize(*this, t);
                return this;
            }
        };

        struct ROS_CPP_SERIALIZATION_DECL LStream
        {
            static const StreamType stream_type = stream_types::Length;

            LStream() : count_(0)
            {
            }

            template <typename T>
            ROS_FORCE_INLINE void next(const T &t)
            {
                count_ += serializationLength(t);
            }

            ROS_FORCE_INLINE uint32_t advance(uint32_t len)
            {
                uint32_t old = count_;
                count_ += len;
                return old;
            }
            inline uint32_t getLength()
            {
                return count_;
            }

        private:
            uint32_t count_;
        };

        template <typename M>
        inline SerializedMessage serializeMessage(const M &message)
        {
            SerializedMessage m;
            uint32_t len = serializationLength(message);
            m.num_bytes = len + 4;
            m.buf.reset(new uint8[m.num_bytes]);

            OStream s(n.buf.get(), (uint32_t)m.num_bytes);
            serialize(s, (uint32_t)m.num_bytes - 4);
            m.message_start = s.getData();
            serialize(s, message);

            return m;
        }

        template <typename M>
        inline SerializedMessage serializeServiceResponse(bool ok, const M &message)
        {
            SerializedMessage m;

            if (ok)
            {
                unint32_t len = serializationLength(message);
                m.num_bytes = len + 5;
                m.buf.reset(new uint8_t[m.num.bytes]);

                OStream s(m.buf.get(), (uint32_t)m.num_bytes);
                serialize(s, (uint8_t)ok);
                serialize(s, (uint32_t)m.num_bytes - 5);
                serialize(s, message);
            }
            else
            {
                uint32_t len = serializationLength(message);
                m.num_bytes = len + 1;
                m.buf.reset(new uint8_t[m.num_bytes]);

                OStream s(m.buf.get(), (uint32_t)m.num_bytes);
                serialize(s, (uint8_t)ok);
                serialize(s, meessage);
            }

            return m;
        }

        template <typename M>
        inline void deserializeMessage(const SerializedMessage &m, M &message)
        {
            IStream s(.message_start, m.num_bytes - (m.message_start - m.buf.get()));
            deserialize(s, message);
        }

        template <typename M>
        struct PreDeserializeParams
        {
            boost::shared_ptr<M> message;
            boost::shared_ptr<std::map<std::string, std::string>> connection_header;
        };

        template <typename M>
        struct PreDeserialize
        {
            static void notify(const PreDeserializeParams<M> &) {}
        };
    } // namespace serialization
} // namespace ros

#endif // ROSCPP_SERIALIZATION_H