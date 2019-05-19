/**
*@file: server.cpp
*@brief: 提供服务的节点
*@exe_name: server
*@node_name: greetings_server
*@service: greetings
*/

/**头文件*/
#include <ros/ros.h>
#include <service_demo/Greeting.h>

/**服务处理函数
*@param: req请求数据体的指针
*@param: res结果数据体的指针
*/
bool handle_function(service_demo::Greeting::Request &req,
                     service_demo::Greeting::Response &res) 
{
  //显示请求信息
  ROS_INFO("Request from %s with age %d", req.name.c_str(), req.age);
  //处理请求， 结果写入response
  res.feedback = "Hi " + req.name + ".I'm server !";
  //返回true， 正确处理了请求
  return true;
}

/**主函数*/
int main(int argc, char **argv) {
  ros::init(argc, argv, "greetings_server"); //解析参数， 命名节点
  ros::NodeHandle nh;                        //创建句柄， 实例化node
  // 写明服务的处理函数
  ros::ServiceServer service = nh.advertiseService("greetings", handle_function);
  // 阻塞，检测服务请求并处理
  ros::spin();
  return 0;
}