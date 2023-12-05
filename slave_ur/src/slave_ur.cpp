#include"ros/ros.h"
#include"trajectory_msgs/JointTrajectory.h"
#include"trajectory_msgs/JointTrajectoryPoint.h"
#include "sensor_msgs/JointState.h"
#include"message_filters/subscriber.h"
#include"message_filters/sync_policies/approximate_time.h"
#include "cartesian_state_msgs/PoseTwist.h"
#include "geometry_msgs/WrenchStamped.h"
using namespace std;
double joint_state[6]={0.0};
double joint_values[6]={0.0};
double robot_control[6]={0.0};
vector<double> joint_goal={0.0,0.0,0.0,0.0,0.0,0.0};
bool singal=true; 
 ros::Publisher pub;
 ros::Time last_publish_time;
 ros::Duration period;
double end_position[8]={0.0};
 std::string topic_arm_state;
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
// std::cout<<"robot_joint.position[0]"<<robot_joint.position[0]<<std::endl;
// std::cout<<"robot_joint.position[1]"<<robot_joint.position[1]<<std::endl;
// std::cout<<"robot_joint.position[2]"<<robot_joint.position[2]<<std::endl;
// std::cout<<"robot_joint.position[3]"<<robot_joint.position[3]<<std::endl;
// std::cout<<"robot_joint.position[4]"<<robot_joint.position[4]<<std::endl;
// std::cout<<"robot_joint.position[5]"<<robot_joint.position[5]<<std::endl;
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

         joint_goal[0]=slave_joint[2]+robot_joint[0];
         joint_goal[1]=slave_joint[1]+robot_joint[1];
         joint_goal[2]=slave_joint[0]+robot_joint[2];
         joint_goal[3]=slave_joint[3]+robot_joint[3];
         joint_goal[4]=slave_joint[4]+robot_joint[4];
         joint_goal[5]=slave_joint[5]+robot_joint[5];
        //  joint_goal[0]=slave_joint[0];
        //  joint_goal[1]=slave_joint[1];
        //  joint_goal[2]=slave_joint[2];
        //  joint_goal[3]=slave_joint[3];
        //  joint_goal[4]=slave_joint[4];
        //  joint_goal[5]=slave_joint[5];
         if(joint_goal[0]!=0&&joint_goal[1]!=0&&joint_goal[2]!=0&&joint_goal[3]!=0&&joint_goal[4]!=0&&joint_goal[5]!=0)
         {
std::cout<<"joint_goal[0]"<<joint_goal[0]<<std::endl;
std::cout<<"joint_goal[1]"<<joint_goal[1]<<std::endl;
std::cout<<"joint_goal[2]"<<joint_goal[2]<<std::endl;
std::cout<<"joint_goal[3]"<<joint_goal[3]<<std::endl;
std::cout<<"joint_goal[4]"<<joint_goal[4]<<std::endl;
std::cout<<"joint_goal[5]"<<joint_goal[5]<<std::endl;
    trajectory_msgs::JointTrajectoryPoint point;
    // point.positions.push_back(1.382300);
    // point.positions.push_back(-1.477011);
    // point.positions.push_back(-2.914310);
    // point.positions.push_back(-2.690375);
    // point.positions.push_back(-1.784731);
    // point.positions.push_back(2.249092);
    point.positions.push_back(joint_goal[0]);
    point.positions.push_back(joint_goal[1]);
    point.positions.push_back(joint_goal[2]);
    point.positions.push_back(joint_goal[3]);
    point.positions.push_back(joint_goal[4]);
    point.positions.push_back(joint_goal[5]);
    trajectory_msgs::JointTrajectory msg;
        msg.joint_names.push_back("elbow_joint");
        msg.joint_names.push_back("shoulder_lift_joint");
        msg.joint_names.push_back("shoulder_pan_joint");
        msg.joint_names.push_back("wrist_1_joint");
        msg.joint_names.push_back("wrist_2_joint");
        msg.joint_names.push_back("wrist_3_joint");
       
        ros::Time current_time=ros::Time::now();
        ros::Duration time_interval=current_time-last_publish_time;
        point.time_from_start=ros::Duration(1.0,0);
    msg.points.push_back(point);
    pub.publish(msg);
    //   last_publish_time = current_time;
         }
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
        motionjointcontrol(robot_control,joint_state);
}

void state_arm_callback(const cartesian_state_msgs::PoseTwistConstPtr msg)
{
    //   arm_real_position_ << msg->pose.position.x, msg->pose.position.y,
    //                  msg->pose.position.z;

//   arm_real_orientation_.coeffs() << msg->pose.orientation.x,
//                                msg->pose.orientation.y,
//                                msg->pose.orientation.z,
//                                msg->pose.orientation.w;
      if(singal)
    {
                               end_position[0]=msg->pose.position.x;
                               end_position[1]=msg->pose.position.y;
                               end_position[2]=msg->pose.position.z;
                               end_position[3]=msg->pose.orientation.x;
                               end_position[4]=msg->pose.orientation.y;
                               end_position[5]=msg->pose.orientation.z;
                               end_position[6]=msg->pose.orientation.w;
                               singal=false;
    }
std::cout<<"pose"<<msg->pose.position.x<<std::endl;
std::cout<<"pose"<<msg->pose.position.y<<std::endl;
std::cout<<"pose"<<msg->pose.position.z<<std::endl;
std::cout<<"pose_orientation"<<msg->pose.orientation.x<<std::endl;
std::cout<<"pose_orientation"<<msg->pose.orientation.y<<std::endl;
std::cout<<"pose_orientation"<<msg->pose.orientation.z<<std::endl;
std::cout<<"pose_orientation"<<msg->pose.orientation.w<<std::endl;
}








int main(int argc,char**argv){
    ros::init(argc,argv,"ur_joint_publisher");
    ros::NodeHandle n;
     pub=n.advertise<trajectory_msgs::JointTrajectory>("/scaled_pos_joint_traj_controller/command",10);
    // ros::Subscriber sub_arm_state_ = n.subscribe("/topic_arm_state", 10, state_arm_callback);
//   ros::Subscriber  sub_arm_state_ = n.subscribe("/scaled_pos_joint_traj_controller/follow_joint_trajectory/action_topic", 10,
//                                  state_arm_callback,
//                                  ros::TransportHints().reliable().tcpNoDelay());
    ros::Subscriber hand_pose_subscriber =n.subscribe("/phantom/phantom/joint_states",1,handposeCallback);
    ros::Subscriber robot_jonit_subsciber =n.subscribe("/joint_states",1,robotjointcallback);
//     std::cout<<"joint_goal[0]"<<joint_goal[0]<<std::endl;
// std::cout<<"joint_goal[1]"<<joint_goal[1]<<std::endl;
// std::cout<<"joint_goal[2]"<<joint_goal[2]<<std::endl;
// std::cout<<"joint_goal[3]"<<joint_goal[3]<<std::endl;
// std::cout<<"joint_goal[4]"<<joint_goal[4]<<std::endl;
// std::cout<<"joint_goal[5]"<<joint_goal[5]<<std::endl;
    // trajectory_msgs::JointTrajectoryPoint point;
    // point.positions.push_back(1.382300);
    // point.positions.push_back(-1.477011);
    // point.positions.push_back(-2.914310);
    // point.positions.push_back(-2.690375);
    // point.positions.push_back(-1.784731);
    // point.positions.push_back(2.249092);
    // point.positions.push_back(joint_goal[0]);
    // point.positions.push_back(joint_goal[1]);
    // point.positions.push_back(joint_goal[2]);
    // point.positions.push_back(joint_goal[3]);
    // point.positions.push_back(joint_goal[4]);
    // point.positions.push_back(joint_goal[5]);
    // trajectory_msgs::JointTrajectory msg;
    //     msg.joint_names.push_back("shoulder_pan_joint");
    //     msg.joint_names.push_back("shoulder_lift_joint");
    //     msg.joint_names.push_back("elbow_joint");
    //     msg.joint_names.push_back("wrist_1_joint");
    //     msg.joint_names.push_back("wrist_2_joint");
    //     msg.joint_names.push_back("wrist_3_joint");
       
    //     ros::Time current_time=ros::Time::now();
    //     ros::Duration time_interval=current_time-last_publish_time;
    //     point.time_from_start=ros::Duration(1.0,0);
    //     while(pub.getNumSubscribers()==0)
    //     {
    //         ros::Duration(0.1).sleep();
    //     }
    // msg.points.push_back(point);
    // pub.publish(msg);
    // // ros::AsyncSpinner spinner(2);
    // // spinner.start();
    ros::spin();
return 0;
}
