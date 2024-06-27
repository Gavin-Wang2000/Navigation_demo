// 功能测试： 向/cmd_vel发送数据
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "sub_test");                                       // 初始化ros节点
    ros::NodeHandle nh;                                                      // 创建节点句柄
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10); // 创建一个Publisher，发布名为/cmd_vel的topic，消息类型为geometry_msgs::Twist，队列长度10
    geometry_msgs::Twist msg;                                                // 实例化消息对象
    msg.linear.x = 1.0;
    msg.angular.z = 1.0;
    ros::Rate r(1);   // 设置消息发布频率
    while (ros::ok()) // 和rospy.is_shutdown()功能一样
    {
        pub.publish(msg); // 发布消息
        ROS_INFO("发布消息:linear.x = %f, angular.z = %f", msg.linear.x, msg.angular.z);
        r.sleep(); // 按前面设置的频率延时
        ros::spinOnce();
    }

    return 0;
}
