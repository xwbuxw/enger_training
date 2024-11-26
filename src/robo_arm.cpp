#include "geometry_msgs/Pose.h"
#include "ros/console.h"
#include "std_msgs/ColorRGBA.h"
#include <robo_arm.h>
#include <vector>
#include <nav_msgs/Odometry.h>

class RobotArm {
public:
    RobotArm(){
        arm_pub = nh.advertise<nav_msgs::Odometry>("/arm_ctrl", 10);
        arm_sub = nh.subscribe<nav_msgs::Odometry>("/arm_read",10, &RobotArm::arm_pose_cb,this);
        arm_pose init_pose = { //TODO 待修改，与kxd约定初始位置
        .x = 0.0,
        .y = 0.0,
        .z = 0.0,
        .cam_angle = 0.0,
        .paw_angle = 0.0
        };
        arm_control.push_back(init_pose);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher arm_pub;
    ros::Subscriber arm_sub;

    struct arm_pose{
        double x;
        double y;
        double z;
        double cam_angle;
        double paw_angle;
    };

    arm_pose current_arm_pose;

    std::vector<arm_pose> arm_control;

    void arm_pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
    {
    nav_msgs::Odometry target_arm_pose_sub;
    target_arm_pose_sub = *msg;
    current_arm_pose.cam_angle = target_arm_pose_sub.pose.pose.position.x;
    current_arm_pose.x = target_arm_pose_sub.pose.pose.orientation.x;
    current_arm_pose.y = target_arm_pose_sub.pose.pose.orientation.y;
    current_arm_pose.z = target_arm_pose_sub.pose.pose.orientation.z;
    current_arm_pose.paw_angle = target_arm_pose_sub.pose.pose.orientation.w;
    //ROS_INFO("x=%.2f,y=%.2f,z=%.2f",cur_pose.pose.pose.position.x,cur_pose.pose.pose.position.y,cur_pose.pose.pose.position.z);
    };

    void arm_pose_pub(){
        for (size_t i = 0; i < arm_control.size(); ++i) {
            nav_msgs::Odometry target_arm_pose_pub;
            while(arm_arrived(arm_control[i]) == false){
            target_arm_pose_pub.pose.pose.position.x = arm_control[i].cam_angle; // 注意这里应该是 num.x 而不是 arm_control[num].cam_angle
            target_arm_pose_pub.pose.pose.orientation.x = arm_control[i].x;
            target_arm_pose_pub.pose.pose.orientation.y = arm_control[i].y;
            target_arm_pose_pub.pose.pose.orientation.z = arm_control[i].z;
            target_arm_pose_pub.pose.pose.orientation.w = arm_control[i].paw_angle;
            arm_pub.publish(target_arm_pose_pub); // 修正为 target_arm_pose_pub
            ROS_INFO("arm_moving_to_target");
            ros::Duration(0.1).sleep();
            }
        }
    };

    bool arm_arrived(arm_pose target_arm_pose){
        if(fabs(current_arm_pose.x - target_arm_pose.x)<0.3 &&
        fabs(current_arm_pose.y - target_arm_pose.y)<0.3 &&
        fabs(current_arm_pose.z - target_arm_pose.z)<0.3 &&
        fabs(current_arm_pose.cam_angle - target_arm_pose.cam_angle)<0.3 &&
        fabs(current_arm_pose.paw_angle - target_arm_pose.paw_angle)<0.3
        ){
            return true;
        }else{
            return false;
        }
    }


};