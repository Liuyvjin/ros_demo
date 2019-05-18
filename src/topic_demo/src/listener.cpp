/**
 * @file: listener.cpp
 * @breif: 订阅接收gps_message话题的消息 
 * @exe_name: listener_sub
 * @node_name: listener
 * @author: liujin
 * @date: 2019/5/18
 */

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "topic_demo/gps.h"
/**订阅话题的回调函数*/
void chatterCallback(const topic_demo::gps::ConstPtr& msg)
{
	std_msgs::Float32 distance;
	distance.data = sqrt(pow(msg->x,2)+pow(msg->y,2));
	// 输出计算的距离
	ROS_INFO("Listener: Distance to origin is %f, state: %s", \
	distance.data,msg->state.c_str());
}

/**主函数*/
int main(int argc, char *argv[])
{

	ros::init(argc, argv, "listener");

	ros::NodeHandle n;
	/**
	 第二个参数是消息队列的长度。如果消息到达的比处理的快，，则此处的数字指定在丢弃之前
	 要缓冲的消息数量。
	 */
	ros::Subscriber sub = n.subscribe("gps_message", 10, chatterCallback);

	//反复调用当前可触发的回调函数，阻塞。
	ros::spin();
	return 0;
}