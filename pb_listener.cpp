#include <ros/protobuffer_traits.h>
#include <ros/serialization_protobuffer.h>

#include "ros/ros.h"
#include "publish_info.pb.h"

void chatterCallback(const ros::MessageEvent<superbai::sample::PublishInfo> &msg) {

    /**
     * 提问：这里的msg.getMessage()方法是在哪里定义的
     * 回答：
     * msg.getMessage() 方法是在 ros::MessageEvent 这个模板类中定义的。
     * getMessage() 返回一个 boost::shared_ptr<M>，即指向消息体的智能指针。
     * 你可以通过这个指针访问消息的成员函数，比如 protobuf 的 DebugString()。
     */
    std::cerr << "I heard: " << msg.getMessge()->debugString() << std::endl;

    /**
     * 提问：这里是不是用到了模板元编程
     * 回答：是的，这里确实用到了模板元编程。
     * 其作用是：根据不同的消息类型，自动获取该类型的消息定义，而且这一切都是在编译期完成类型推导的。
     * 详细解释：
     * ros::message_traits::Definition<T> 是一个模板结构体，T 是模板参数。
     * 这里传入的 T 是 superbai::sample::PublishInfo，即你的 protobuf 消息类型。
     * .value() 是 Definition<T> 结构体的静态成员函数（或静态成员变量），用于获取类型 T 的消息定义字符串。
     */
    std::string def = 
        ros::message_traits::Definition<superbai::sample::PublishInfo>::value();
    std::cout << "def: " << def << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pb_listener");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/Sorbai", 1000, chatterCallback);

    ros::spin();

    return 0;
}