/**
* @file: param.cpp
* @brief: 演示ros提供的参数服务器的使用，
*         包括ros::NodeHandle和ros::param两套api
*/

/**头文件*/
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "param_demo");
  ros::NodeHandle nh;

  /// 定义示例用参数
  int parameter1, parameter2, parameter3, parameter4, parameter5;
  
  //############# Get Param的三种方法
  //① ros::param::get()获取参数“param1”的value，写入到parameter1上
  bool ifget1 = ros::param::get("param1", parameter1);//返回代表成功与否bool值
  //② ros::NodeHandle::getParam()获取参数， 与①作用相同
  bool ifget2 = nh.getParam("param2", parameter2);
  //③ ros::NodeHandle::param()类似于①和②
  //但如果get不到指定的param， 它可以给param指定一个默认值(如33333)
  nh.param("param3", parameter3, 33333);
  // param是否取得
  if (ifget1){ROS_INFO("Get param1 success");} 
        
  //############# Set Param
  //① ros::param::set()设置参数
  parameter4 = 4;
  ros::param::set("param4", parameter4);
  //② ros::NodeHandle::setParam()设置参数
  parameter5 = 5;
  nh.setParam("param5", parameter5);
  
  //############# Check Param
  //① ros::NodeHandle::hasParam()
  bool ifparam5 = nh.hasParam("param5");
  //② ros::param::has()
  bool ifparam6 = ros::param::has("param6");
  
  //############# Delete Param
  //① ros::NodeHandle::deleteParam()
  bool ifdeleted5 = nh.deleteParam("param5");
  //② ros::param::del()
  bool ifdeleted6 = ros::param::del("param6");
}