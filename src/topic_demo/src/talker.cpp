/**
 * @file: talker.cpp
 * @breif: 发送消息到gps_message话题
 * @node_name: talker
 * @author: liujin
 * @date: 2019/5/18
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
//msg头文件
#include "topic_demo/gps.h"
#define random(x) (rand()%x)

int main(int argc, char *argv[])
{
	// main函数第一个执行的函数就是它，是为了向master注册该节点。
    // 函数需要查看argc和argv，以便它可以执行命令行提供的任何ROS
	// 参数和名称重映射。 对于程序化重映射，您可以使用不同版本的
	// init（）直接重映射，但对于大多数命令行程序，传递argc和
	// argv是最简单的方法。 init（）的第三个参数是节点的名称。 
	// 在使用ROS系统的任何其他部分之前，必须调用其中一个版本的ros :: init（）。
	ros::init(argc, argv, "talker");

	/**
	 节点的句柄类，该类实例化的对象，可以控制这个节点，
	 是与ROS系统通信的主要访问点。 构造的第一个NodeHandle
	 将完全初始化该节点，并且最后一个被破坏的NodeHandle将关闭该节点。
	 */
	ros::NodeHandle n;

	/**
	 advertise（）函数是用来告诉ROS您要在给定的主题名称上发布消
	 息。这将触发对ROS主节点的调用，主节点保留一个注册表（谁正
	 在发布以及谁在订阅）。 在进行此advertise（）调用之后，主节点
	 将通知尝试订阅此主题名称的任何人，并且他们将轮流协商与此节
	 点进行对等连接。 advertise（）返回一个Publisher对象A，允许您
	 通过调用A.publish（）来发布有关该主题的消息。 一旦销毁了返回
	 的Publisher对象的所有副本，该主题将自动被撤销。 
	 advertise（）的第二个参数是发布消息的消息队列的大小。 如果消
	 息的发布速度比发送消息的速度快，则此处的数字指定在丢弃之前
	 要缓冲的消息数量。（一般一个较小的值就行）
	**/
	ros::Publisher gps_pub = n.advertise<topic_demo::gps>("gps_message", 10);

	///用来控制循环频率的对象，1Hz
	ros::Rate loop_rate(1);

	/**
	 * 这是用来记录多少条消息已经被发送出去
	 * 用来保证每条消息都不同
	 */
	int count = 0;
	while (ros::ok())
	{
		//构造消息
		topic_demo::gps msg;
		std::stringstream ss;
		ss << "message No." << count;	
		msg.state = ss.str();
		msg.x = random(100);
		msg.y = random(100);
		// 输出信息
		ROS_INFO("Talker:x = %f , y = %f ,%s", msg.x, msg.y,msg.state.c_str());
		// 发布消息
		gps_pub.publish(msg);
		
		//检查订阅的话题消息队列是否有消息，有的话调用一次回调函数，在这里有何没有一样
		//因为没有订阅
		ros::spinOnce(); 

		//使用刚刚定义的对象控制循环时间
		loop_rate.sleep();
		
		//更新count
		++count;
	}

	return 0;
}