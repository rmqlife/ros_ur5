
#include "ros/ros.h"  
#include "sunrise/sriCommDefine.h"
#include "sunrise/sriCommManager.h"
#include "fstream"
#include "std_msgs/String.h" //ros标准消息里面的字符串消息
#include <geometry_msgs/WrenchStamped.h>
#include<iostream>
#include <sstream>
using namespace std;
#define WRENCH_TOPIC    "/phantom/phantom/force_feedback"

// bool  CSRICommManager::OnCommM8218(float fx, float fy, float fz, float mx, float my, float mz)
// {

// 	printf("M8218 = %f, %f, %f,   %f, %f, %f\n", fx, fy, fz, mx, my, mz);
// 		return true;
// }
	// float sixnumber()
	// {


	// }

int main(int argc,char* argv[])
{
	float  *receive_value;
	printf("SRI TCP Client Demo.\n");

	CSRICommManager commManager;
	//ROS节点初始化
ros::init(argc, argv,  "wrench_signal_generate");
ros::NodeHandle n;

 ros::Publisher wrench_pub = n.advertise<geometry_msgs::WrenchStamped>(WRENCH_TOPIC, 5);
	//创建ROS节点句柄
	

	//创建一个Publisher,发布turtle/cmd_vel话题，消息类型为geometry_msgs::Twist,队列长度为10

// ros::Publisher pub=n.advertise<std_msgs::String >("/wrench",10);

//创建被发布的信息
geometry_msgs::WrenchStamped wrench_msg;
// std_msgs::String msg;
/**
	 * 循环频率10Hz
	 */
ros::Duration Sleep(5.0);
 Sleep.sleep();
 ros::Rate   loop_rate(0.5);
 if (commManager.Init() == true)
	 {
				   if (commManager.Run() == true)
		{	
		}
	 }
ofstream outfile;
outfile.open("/home/duan/catkin_ws/output_sensor.txt");
while (ros::ok())
// while(true)
	{
		/**
		 * 先创建一个消息对象。然后用数据填充它，最后发布它。
		 * 要查看消息结构在命令行使用rosmsg show std_msgs/String，其它消息类似
		 */
						// while (true)
						// {
						// 	getchar();
						// }
				// 关闭文档
			
	
					receive_value=commManager.output;

					// printf("M128=%f,%f,%f,%f,%f,%f\n",receive_value[0],receive_value[1],receive_value[2],receive_value[3],receive_value[4],receive_value[5]);
                wrench_msg.wrench.force.x= receive_value[0];
				wrench_msg.wrench.force.y= receive_value[2];
				wrench_msg.wrench.force.z= receive_value[1];
std::cout<<"wrench_msg.wrench.force.z= "<<wrench_msg.wrench.force.y<<std::endl;
std::cout<<"wrench_msg.wrench.force.y= "<<wrench_msg.wrench.force.z<<std::endl;
std::cout<<"wrench_msg.wrench.force.x= "<<wrench_msg.wrench.force.x<<std::endl;
// outfile.open("dataFile.txt",ofstream::app);
// fstream file("dataFile.txt",ios::out);
    //  outfile<<"fZ: "<< receive_value[2]<<endl;
            //    std::cout << wrench_msg.wrench.force.z << std::endl;
				wrench_pub.publish(wrench_msg);

				//rate.sleep();
		
	}
	outfile.close();
printf("Demo done!\nPress ENTER to close.\n");

	getchar();
return 0;
}

