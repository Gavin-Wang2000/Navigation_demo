#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>         // 引用move_base的信息
#include <actionlib/client/simple_action_client.h> // 引用actionlib库
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <signal.h> // 引用signal头文件，为了做节点退出操作

using namespace std;
// 定义一个SimpleActionClient，用来给move_base一个目标点：
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// 目标点信息位置规划
move_base_msgs::MoveBaseGoal target; // 定义目标点位置
int target_index = 0;                // 定义目标点索引
double target_position_x[1000] = {0, -0.8, 0.3, 3.1, 4.8, 7.8, 7.8};
double target_position_y[1000] = {0, 1.7, 4.0, 6.9, 7.6, 7.4, 5.2};


void DoShutdown(int sig)
{
    // 这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
    ROS_INFO("shutting down!");
    ros::shutdown();
    exit(sig); // 为了更完整，处理一下退出的signal
}

void done_cb(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result)
{
    // 判断是否执行成功：
    if (state.state_ == state.SUCCEEDED)
    {
        ROS_INFO("响应成功");
    }

    else
        ROS_INFO("服务器连接失败");
}

void action_cb()
{
    ROS_INFO("客户端与服务端连接成功");
}

void feedback_cb(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback, MoveBaseClient *ac)
{
    double distance = sqrt(std::pow(target_position_x[target_index] - feedback->base_position.pose.position.x, 2) + std::pow(target_position_y[target_index] - feedback->base_position.pose.position.y, 2));

    if (distance < 2)
    {
        ROS_INFO("到达目标点，切换下一个目标点,%d",target_index);
        target_index += 1;
        if (target_index > 7)
            target_index = 0;
        ac->cancelGoal();
    }
    else
    {
        ROS_INFO("当前 %.2f,%.2f 距离差:%.2f", feedback->base_position.pose.position.x,
                 feedback->base_position.pose.position.y,
                 distance);
    }
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "a_goals_sender"); // 初始化ros节点
    MoveBaseClient ac("move_base", true);    // 定义服务器对象
    // MoveBaseClient ac("move_base", true);
    // 等待客户端响应
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    // 声明一个目标点goal，注意MoveBaseGoal的格式：
    move_base_msgs::MoveBaseGoal goal;

    // 在ctrl+c时有效执行退出操作，方便扩展（参见参考【3】）
    signal(SIGINT, DoShutdown);

    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    // 初始目标点给予
    target.target_pose.pose.position.x = target_position_x[0];
    target.target_pose.pose.position.y = target_position_y[0];

    while (ros::ok())
    {
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        // 以下是一个随意取的二维目标点：
        goal.target_pose.pose.position.x = target_position_x[target_index];
        goal.target_pose.pose.position.y = target_position_y[target_index];
        goal.target_pose.pose.orientation.w = 1.0;
        ROS_INFO("Sending goal");
        // 定义好了goal，就可以调用SimpleActionClient的现成方法sendGoal()，非常方便：
        ac.sendGoal(goal, &done_cb, &action_cb, boost::bind(&feedback_cb, _1, &ac));
        ac.waitForResult(); // 等待服务器响应
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
