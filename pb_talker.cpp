#include <ros/protobuffer_traits.h>
#include <ros/serialization_protobuffer.h>
#include "ros/ros.h"
#include "publish_info.pb.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "pb_talker");

    /**
     * 把当前节点的日志级别设置为Debug
     * 这样在代码里ROS——DEBUG()打印的调试信息也会显示出来
     * 默认只显示INFO及以上的级别
     */
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::concole::levels::Debug);

    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<superbai::sample::PublishInfo>("/Sorbai", 1000);

    ros::Rate loop_rate(10);

    superbai::sample::publishInfo proto_msg_info;
    proto_msg_info.set_name("sorbai");
    proto_msg_info.set_message_type("test_message");
    proto_msg_info.set_publish_msg("sorbai is a bilibili up");

    int count = 0;
    while (ros::ok()) {
        pub.publish(proto_msg_info);
        std::cerr << "DebugMsg: " << proto_msg_info.DebugString() << std::endl;
        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
}