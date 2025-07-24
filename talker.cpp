#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "talker");

    ros::Publisher chatter_pub = n.advertise<std_msgs::String> ("chatter", 1000);

    ros::Rate loop_rate(10);

    int count = 0;

    while (ros::ok()) {
        std_msgs::String msg;
        ss << "hello world" << count;

        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
        chatter_pub.publish(msg);

        /**
         * 这里用 ros::spinOnce()，是因为你需要每次循环都能继续往下执行，而不是卡在 spin() 里。
         * 如果用 ros::spin()，主线程会被阻塞，后面的发布和休眠都不会执行。
         */
        ros::spinOnce();

        /**
         * 计算实际耗时：从上次调用 sleep() 到当前的时间间隔。
         * 计算需要休眠的时间：例如，设定 10Hz（周期 100ms），若循环体执行耗时 60ms，则休眠 40ms。
         */
        loop_rate.sleep();

        ++count;
    }

    return 0;
}