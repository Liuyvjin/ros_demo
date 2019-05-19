/**
*@file: client.cpp
*@brief: 调用服务的节点
*@exe_name: client
*@node_name: greetings_client
*@service: greetings
*/

/**头文件*/
#include "ros/ros.h"
#include "service_demo/Greeting.h"

/**主函数*/
int main(int argc, char **argv) 
{
  ros::init(argc, argv,"greetings_client"); // 初始化，节点命名为"greetings_client"
  ros::NodeHandle nh;
  
  // 定义service客户端， service名字为“greetings”， service类型为Service_demo
  ros::ServiceClient client =
      nh.serviceClient<service_demo::Greeting>("greetings");
  
  // 实例化srv， 设置其request消息的内容， 这里request包含两个变量， name和age，
  // 见Greeting.srv
  service_demo::Greeting srv;
  srv.request.name = "liujin";
  srv.request.age = 20;

  // 调用服务
  if (client.call(srv)) {
    // 注意我们的response部分中的内容只包含一个变量response， 另，
    // 注意将其转变成字符串，client.call()返回是否调用成功
    ROS_INFO("Response from Server: %s", srv.response.feedback.c_str());
  }
  else {
    ROS_ERROR("调用服务失败");
    return 1;
  }
  return 0;
}