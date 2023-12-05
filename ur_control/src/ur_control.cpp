#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include<moveit/planning_scene_interface/planning_scene_interface.h> 
#include<moveit/planning_scene_monitor/planning_scene_monitor.h>
#include<moveit/planning_scene/planning_scene.h>
#include "sensor_msgs/JointState.h"
#include<vector>
// #include "robot_msgs/Move.h"
using namespace std;
double joint_state[6]={0.0};
double joint_values[6]={0.0};
double robot_control[6]={0.0};
vector<double> joint_goal={0.0,0.0,0.0,0.0,0.0,0.0};
bool singal=true;
   moveit::planning_interface::MoveGroupInterface* move_group;    
 
void robotjointcallback(const sensor_msgs::JointState& robot_joint)
{
 
if(singal)
{
    robot_control[0]=robot_joint.position[0];
    robot_control[1]=robot_joint.position[1];
    robot_control[2]=robot_joint.position[2];
    robot_control[3]=robot_joint.position[3];
    robot_control[4]=robot_joint.position[4];
    robot_control[5]=robot_joint.position[5];
    singal=false;
}
// cout<<"robot_joint"<<robot_control[0]<<endl;
// cout<<"robot_joint"<<robot_control[1]<<endl;
// cout<<"robot_joint"<<robot_control[2]<<endl;
// cout<<"robot_joint"<<robot_control[3]<<endl;
// cout<<"robot_joint"<<robot_control[4]<<endl;
// cout<<"robot_joint"<<robot_control[5]<<endl;

}

void motionjointcontrol(double* robot_joint,double* slave_joint)
{
    bool init_success=false;
    cout<<"move_group_start"<<endl;
    for(int i=0;i<6;i++)
    {
        if(slave_joint[i]!=0&&robot_joint[i]!=0)
            {
init_success=true;
            }
    }
    
        if(init_success)
        {
        //  joint_goal[0]=slave_joint[0]+robot_joint[0];
        //  joint_goal[1]=slave_joint[1]+robot_joint[1];
         joint_goal[2]=slave_joint[2]+robot_joint[2];
         joint_goal[3]=slave_joint[3]+robot_joint[3];
         joint_goal[4]=slave_joint[4]+robot_joint[4];
         joint_goal[5]=slave_joint[5]+robot_joint[5];
        joint_goal[0]=-3.14;  
         joint_goal[1]=-1.5;
        //  joint_goal[2]=1.57;
        //  joint_goal[3]=-3.14;
        //  joint_goal[4]=-1.57;
        cout<<" joint_goal"<<joint_goal[0]<<endl;
        cout<<" joint_goal"<<joint_goal[1]<<endl;
        cout<<" joint_goal"<<joint_goal[2]<<endl;
        cout<<" joint_goal"<<joint_goal[3]<<endl;
        cout<<" joint_goal"<<joint_goal[4]<<endl;
        cout<<" joint_goal"<<joint_goal[5]<<endl;
    //     ros::NodeHandle nd;
    //     ros::AsyncSpinner spinner(1);
    //    spinner.start();
//  moveit::planning_interface::MoveGroupInterface move_group("manipulator"); 
         cout<<"move_group_start_3"<<endl;
    //   move_group->setPlannerId("EST");
            // ros::Duration(0.8).sleep();
         move_group->setJointValueTarget(joint_goal);
        // ros::Duration(2).sleep();
         moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        //    move_group->setPlanningTime(10);
             cout<<"move_group_start_4"<<endl;
                //  ros::Duration(2).sleep();
         bool success=(move_group->plan(my_plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
                //  ros::Duration(2).sleep();
            cout<<"move_group_start_5"<<endl;
         if(success)
         {
            move_group->execute(my_plan);
        
         }
         else
         {
cout<<"failed to plan motion"<<endl;
         }
            //  executesuccess=true;
        }
}

 void handposeCallback(const sensor_msgs::JointState& state_msg)
 {



    joint_state[0]=state_msg.position[0];
    joint_state[1]=state_msg.position[1]-0.26889;
    joint_state[2]=state_msg.position[2]+0.639702;
    joint_state[3]=state_msg.position[3]-3.14276;
    joint_state[4]=state_msg.position[4]+3.15091;
    joint_state[5]=state_msg.position[5]+3.41783;
    std::cout<<"joint_state[0]"<<joint_state[0]<<std::endl;
    std::cout<<"joint_state[1]"<<joint_state[1]<<std::endl;
    std::cout<<"joint_state[2]"<<joint_state[2]<<std::endl;
    std::cout<<"joint_state[3]"<<joint_state[3]<<std::endl;
    std::cout<<"joint_state[4]"<<joint_state[4]<<std::endl;
    std::cout<<"joint_state[5]"<<joint_state[5]<<std::endl;
        motionjointcontrol(joint_state,robot_control);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv ,"moveit_publisher");
    ros::NodeHandle n;
    ros::Subscriber hand_pose_subscriber =n.subscribe("/phantom/phantom/joint_states",1,handposeCallback);
    ros::Subscriber robot_jonit_subsciber =n.subscribe("/joint_states",1,robotjointcallback);
   ros::AsyncSpinner spinner(2);
       spinner.start();
  move_group =new moveit::planning_interface::MoveGroupInterface("manipulator");
        //  joint_goal[0]=-3.14;  
        //  joint_goal[1]=-1.5;
        //  joint_goal[2]=1.57;
        //  joint_goal[3]=-3.14;
        //  joint_goal[4]=-1.57;
        //  move_group.setJointValueTarget(joint_goal);

    ros::waitForShutdown();
    return 0;
}