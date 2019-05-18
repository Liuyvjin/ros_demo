/**
* @file: helloworld.cpp
* @brief: hello_world例程,创建一个发布者，不断在话题中发布hello world!
*/

/**头文件*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

/**主程序*/
int main(int argc, char **argv)
{
    ros::init(argc,argv,"hello_node");  //向master注册的节点名称
    ros::NodeHandle n;          // 节点句柄，实例化node

    ///在节点下创建一个发布者，向话题hello_message发送消息
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("hello_message",1000);
    ros::Rate loop_rate(1);  // Rate类实例化，频率为1Hz
    
    std_msgs::String msg;  // 声明发送的信息
    //std::stringstream ss;  // stringstream类对象ss，保存“Hello world!”
    //ss << "Hello world!";
    msg.data="Hello world!";
    while(ros::ok())   // 当这个节点还在运行时 
    {
        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}