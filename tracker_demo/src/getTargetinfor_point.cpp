#include <ros/ros.h>
#include <Eigen/Eigen>
#include <deque>

#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"

// #include "mpc/Polynome.h"                   // 自定义消息
#include "std_msgs/Float32MultiArray.h"

#define delta_t 0.05
#define traj_size 10

#define robot_type "scout" // hunter || scout

ros::Publisher traj_pub_nmpc;

// nav_msgs::Odometry target_odom;
nav_msgs::Odometry robot_odom;

Eigen::Vector3d target_state;

bool is_rcv_target_odom = false;
bool is_rcv_robot_odom = false;

void odom1Callbck(const nav_msgs::Odometry& msg){
    if (msg.child_frame_id == "X" || msg.child_frame_id == "O") return;
    is_rcv_robot_odom = true;
    robot_odom = msg;    
}

void goalCallbck(const geometry_msgs::PoseStamped::ConstPtr& msg){
    geometry_msgs::PoseStamped pt = *msg;  
    ROS_WARN("get the goal point successfully !"); 
    Eigen::Quaterniond q(msg->pose.orientation.w, 
        msg->pose.orientation.x, 
        msg->pose.orientation.y, 
        msg->pose.orientation.z);
    Eigen::Matrix3d R(q);
    target_state =  Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, atan2(R.col(0)[1],R.col(0)[0]));
    // std::cout<< target_state<< std::endl; //test
}

void pub_traj_nmpc(Eigen::Vector3d target_goal){
    std_msgs::Float32MultiArray waypoint_array;
    for(int i = 0; i < traj_size; i++)
    {
        waypoint_array.data.push_back(target_goal[0]);
        waypoint_array.data.push_back(target_goal[1]);
        waypoint_array.data.push_back(target_goal[2]);
    }
    traj_pub_nmpc.publish(waypoint_array);
}

void stateCallback(const ros::TimerEvent& e){
    pub_traj_nmpc(target_state);
}



int main(int argc, char* argv[]){
    ros::init(argc,argv,"tracker_state_machine_Hunter");
    ros::NodeHandle nh;
    ros::Duration(2.0).sleep();
    ROS_WARN("tracker_system init done");

    // traj_pub_lmpc = nh.advertise<mpc::Polynome>("trajectory",3);
    traj_pub_nmpc = nh.advertise<std_msgs::Float32MultiArray>("mpc/traj_point", 3);
    if(robot_type=="hunter"){
        ros::Subscriber odom_sub1 = nh.subscribe("/robot1/ackermann_steering_controller/odom", 50, odom1Callbck);
    }
    else if(robot_type=="scout"){
        ros::Subscriber odom_sub1 = nh.subscribe("/scout/odom", 50, odom1Callbck);
    }
    else ROS_ERROR("there is no odometry input!!");
    ros::Subscriber get_goal = nh.subscribe("/move_base_simple/goal", 10, goalCallbck);    
    ros::Timer state_timer = nh.createTimer(ros::Duration(0.2),stateCallback);
    ros::spin();
    return 0;
}