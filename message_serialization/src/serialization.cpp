#include <ros/serialization.h>

namespace ros
{
namespace serialization
{
void throwStreamOverrun()
{
    throw StreamOverrunException("Buffer Overrun");
}
} // namespace serialization
} // namespace ros