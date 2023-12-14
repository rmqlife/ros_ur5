#include "ros/ros.h"
#include "sunrise/sriCommDefine.h"
#include "sunrise/sriCommManager.h"
#include <fstream>
#include "std_msgs/String.h"
#include <geometry_msgs/WrenchStamped.h>
#include <iostream>
#include <sstream>

#define WRENCH_TOPIC "/phantom/phantom/force_feedback"

int main(int argc, char* argv[]) {
    float *receive_value;
    std::cout << "SRI TCP Client Demo." << std::endl;

    CSRICommManager commManager;

    // Initialize ROS
    ros::init(argc, argv, "wrench_signal_generate");
    ros::NodeHandle n;
    ros::Publisher wrench_pub = n.advertise<geometry_msgs::WrenchStamped>(WRENCH_TOPIC, 5);

    // Create a message for wrench data
    geometry_msgs::WrenchStamped wrench_msg;

    ros::Duration sleep_time(5.0);
    sleep_time.sleep();
    ros::Rate loop_rate(500);

    if (commManager.Init() == true) {
        if (commManager.Run() == true) {
        }
    }

    std::ofstream outfile;
    outfile.open("/home/duan/catkin_ws/output_sensor.txt");

    while (ros::ok()) {
        receive_value = commManager.output;

        wrench_msg.wrench.force.x = receive_value[0];
        wrench_msg.wrench.force.y = receive_value[1];
        wrench_msg.wrench.force.z = receive_value[2];
        wrench_msg.wrench.torque.x = receive_value[3];
        wrench_msg.wrench.torque.y = receive_value[4];
        wrench_msg.wrench.torque.z = receive_value[5];

        std::cout << "Force XYZ: (" << wrench_msg.wrench.force.x << ", " << wrench_msg.wrench.force.y << ", " << wrench_msg.wrench.force.z << ") ";
    	std::cout << "Torque XYZ: (" << wrench_msg.wrench.torque.x << ", " << wrench_msg.wrench.torque.y << ", " << wrench_msg.wrench.torque.z << ")" << std::endl;

        wrench_pub.publish(wrench_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    outfile.close();
    std::cout << "Demo done!" << std::endl;
    std::cout << "Press ENTER to close." << std::endl;

    getchar();
    return 0;
}
