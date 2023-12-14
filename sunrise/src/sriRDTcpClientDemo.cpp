#include "ros/ros.h"
#include "sunrise/sriCommDefine.h"
#include "sunrise/sriCommManager.h"
#include "std_msgs/String.h"
#include <geometry_msgs/WrenchStamped.h>
#include <iostream>

#define WRENCH_TOPIC "/sunrise/force"

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
    
    int sample_rate = 100;
    ros::Rate loop_rate(sample_rate);  // Set the loop rate to 100 Hz

    if (commManager.Init() == true) {
        if (commManager.Run() == true) {
        }
    }

    int count = 0;
    while (ros::ok()) {
        receive_value = commManager.output;

        wrench_msg.wrench.force.x = receive_value[0];
        wrench_msg.wrench.force.y = receive_value[1];
        wrench_msg.wrench.force.z = receive_value[2];
        wrench_msg.wrench.torque.x = receive_value[3];
        wrench_msg.wrench.torque.y = receive_value[4];
        wrench_msg.wrench.torque.z = receive_value[5];

        count++;
        if (count >sample_rate){
            count = count - sample_rate;
            std::cout << "Force XYZ: (" << wrench_msg.wrench.force.x << ", " << wrench_msg.wrench.force.y << ", " << wrench_msg.wrench.force.z << ") ";
            std::cout << "Torque XYZ: (" << wrench_msg.wrench.torque.x << ", " << wrench_msg.wrench.torque.y << ", " << wrench_msg.wrench.torque.z << ")" << std::endl;
        }
        wrench_pub.publish(wrench_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    std::cout << "Press ENTER to close." << std::endl;

    getchar();
    return 0;
}
