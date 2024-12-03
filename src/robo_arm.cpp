#include "geometry_msgs/Pose.h"
#include "ros/console.h"
#include "std_msgs/ColorRGBA.h"
#include <robo_arm.h>
#include <unistd.h>
#include <vector>
#include <nav_msgs/Odometry.h>

arm_pose current_arm_pose;
std::vector<arm_pose> arm_control;

void RobotArm::test() {
    ROS_INFO("arm begin testen");
    int temp_index = add_arm_pose(BULE_X+25, BULE_Y, CAR_HIGHT, CIRCULAR_BLUE_CAM_ANGLE, PAW_OPEN);
    //add_arm_pose(BULE_X, BULE_Y, CAR_HIGHT, CIRCULAR_BLUE_CAM_ANGLE, PAW_OPEN);
    add_arm_pose(BULE_X-25, BULE_Y, CAR_HIGHT, CIRCULAR_BLUE_CAM_ANGLE, PAW_OPEN);
     //add_arm_pose(CIRCULAR_RED_X, CIRCULAR_RED_Y, TEKEUP_HEIGHT, CIRCULAR_RED_CAM_ANGLE, PAW_CLOSE);
     //add_arm_pose(430, 0, CAR_HIGHT, CIRCULAR_BLUE_CAM_ANGLE, PAW_OPEN);
    do {
        arm_pose_pub (temp_index);
        temp_index ++;
        ROS_INFO("choose index %d",temp_index);
        sleep(1);
        //usleep(200000);
    }
    while (temp_index < arm_control.size());

    ROS_INFO("arm has testen");
}


void RobotArm::choose (char color) {
    int temp_index=0;
    temp_index = add_arm_pose(INIT_POSE_X, INIT_POSE_Y, CAR_HIGHT, CAM_ANGLE, PAW_OPEN);
    if (color == 'r') {
        ROS_INFO("CHOOSE RED");
        add_arm_pose(RED_X, RED_Y, CAR_HIGHT, CAM_ANGLE_RED, PAW_OPEN);
        add_arm_pose(RED_X, RED_Y, CAR_HIGHT, CAM_ANGLE_RED, PAW_CLOSE);
        add_arm_pose(RED_X, RED_Y, TEKEUP_HEIGHT, CAM_ANGLE_RED, PAW_CLOSE);
    } else if (color == 'g') {
        ROS_INFO("CHOOSE GREEN");
        add_arm_pose(GREEN_X, GREEN_Y, CAR_HIGHT, CAM_ANGLE_GREEN, PAW_OPEN);
        add_arm_pose(GREEN_X, GREEN_Y, CAR_HIGHT, CAM_ANGLE_GREEN, PAW_CLOSE);
        add_arm_pose(GREEN_X, GREEN_Y, TEKEUP_HEIGHT, CAM_ANGLE_GREEN, PAW_CLOSE);
    } else if (color == 'b') {
        ROS_INFO("CHOOSE BLUE");
        add_arm_pose(BULE_X, BULE_Y, CAR_HIGHT, CAM_ANGLE_BLUE, PAW_OPEN);
        add_arm_pose(BULE_X, BULE_Y, CAR_HIGHT, CAM_ANGLE_BLUE, PAW_CLOSE);
        add_arm_pose(BULE_X, BULE_Y, TEKEUP_HEIGHT, CAM_ANGLE_BLUE, PAW_CLOSE);
    }
    
    
    do {
        arm_pose_pub (temp_index);
        if (arm_arrived(arm_control[temp_index])) {
            temp_index ++;
            ROS_INFO("choose index %d",temp_index);
        }
    }
    while (temp_index < arm_control.size());
    ROS_INFO("has chooosen");
}



void RobotArm::put_down(char color){
    int temp_index=0;
    ROS_INFO("begin put down");
    if (color == 'r') {
        ROS_INFO("PUT DOWN RED");
        temp_index = add_arm_pose(CIRCULAR_RED_X, CIRCULAR_RED_Y, TEKEUP_HEIGHT, CAM_ANGLE_RED, PAW_CLOSE);
        add_arm_pose(CIRCULAR_RED_X, CIRCULAR_RED_Y, TEKEUP_HEIGHT, CIRCULAR_RED_CAM_ANGLE, PAW_CLOSE);
        add_arm_pose(CIRCULAR_RED_X, CIRCULAR_RED_Y, CAR_HIGHT, CIRCULAR_RED_CAM_ANGLE, PAW_CLOSE);
        add_arm_pose(CIRCULAR_RED_X, CIRCULAR_RED_Y, CAR_HIGHT, CIRCULAR_RED_CAM_ANGLE, PAW_OPEN);
        add_arm_pose(CIRCULAR_RED_X, CIRCULAR_RED_Y, CAR_HIGHT, CIRCULAR_RED_CAM_ANGLE, PAW_OPEN);
        add_arm_pose(CIRCULAR_RED_X, CIRCULAR_RED_Y, INIT_POSE_Z, CIRCULAR_RED_CAM_ANGLE, PAW_OPEN);
        add_arm_pose(CIRCULAR_RED_X, CIRCULAR_RED_Y, INIT_POSE_Z, CAM_ANGLE, PAW_OPEN);
        add_arm_pose(INIT_POSE_X, INIT_POSE_Y, INIT_POSE_Z, CAM_ANGLE, PAW_OPEN);
    } else if (color == 'g') {
        ROS_INFO("PUT DOWN GREEN");
        temp_index = add_arm_pose(CIRCULAR_GREEN_X, CIRCULAR_GREEN_Y, TEKEUP_HEIGHT, CAM_ANGLE_GREEN, PAW_CLOSE);
        add_arm_pose(CIRCULAR_GREEN_X, CIRCULAR_GREEN_Y, TEKEUP_HEIGHT, CIRCULAR_GREEN_CAM_ANGLE, PAW_CLOSE);
        add_arm_pose(CIRCULAR_GREEN_X, CIRCULAR_GREEN_Y, CAR_HIGHT, CIRCULAR_GREEN_CAM_ANGLE, PAW_CLOSE);
        add_arm_pose(CIRCULAR_GREEN_X, CIRCULAR_GREEN_Y, CAR_HIGHT, CIRCULAR_GREEN_CAM_ANGLE, PAW_OPEN);
        add_arm_pose(CIRCULAR_GREEN_X, CIRCULAR_GREEN_Y, CAR_HIGHT, CIRCULAR_GREEN_CAM_ANGLE, PAW_OPEN);
        add_arm_pose(CIRCULAR_GREEN_X, CIRCULAR_GREEN_Y, INIT_POSE_Z, CIRCULAR_GREEN_CAM_ANGLE, PAW_OPEN);
        add_arm_pose(CIRCULAR_GREEN_X, CIRCULAR_GREEN_Y, INIT_POSE_Z, CAM_ANGLE, PAW_OPEN);
        add_arm_pose(INIT_POSE_X, INIT_POSE_Y, INIT_POSE_Z, CAM_ANGLE, PAW_OPEN);
    } else if (color == 'b') {
        ROS_INFO("PUT DOWN BLUE");
        temp_index = add_arm_pose(CIRCULAR_BLUE_X, CIRCULAR_BLUE_Y, TEKEUP_HEIGHT, CAM_ANGLE_BLUE, PAW_CLOSE);
        add_arm_pose(CIRCULAR_BLUE_X, CIRCULAR_BLUE_Y, TEKEUP_HEIGHT, CIRCULAR_BLUE_CAM_ANGLE, PAW_CLOSE);
        add_arm_pose(CIRCULAR_BLUE_X, CIRCULAR_BLUE_Y, CAR_HIGHT, CIRCULAR_BLUE_CAM_ANGLE, PAW_CLOSE);
        add_arm_pose(CIRCULAR_BLUE_X, CIRCULAR_BLUE_Y, CAR_HIGHT, CIRCULAR_BLUE_CAM_ANGLE, PAW_OPEN);
        add_arm_pose(CIRCULAR_BLUE_X, CIRCULAR_BLUE_Y, CAR_HIGHT, CIRCULAR_BLUE_CAM_ANGLE, PAW_OPEN);
        add_arm_pose(CIRCULAR_BLUE_X, CIRCULAR_BLUE_Y, INIT_POSE_Z, CIRCULAR_BLUE_CAM_ANGLE, PAW_OPEN);
        add_arm_pose(CIRCULAR_BLUE_X, CIRCULAR_GREEN_Y, INIT_POSE_Z, CAM_ANGLE, PAW_OPEN);
        add_arm_pose(INIT_POSE_X, INIT_POSE_Y, INIT_POSE_Z, CAM_ANGLE, PAW_OPEN);
    }

    do {
        arm_pose_pub (temp_index);
        if (arm_arrived(arm_control[temp_index])) {
            temp_index ++;
            ROS_INFO("put down index %d",temp_index);
        }
    }
    while (temp_index < arm_control.size());
    ROS_INFO("has put down");
}


int RobotArm::add_arm_pose(float x, float y, float z, float cam_angle, float paw_angle)
{
    arm_pose temp_pose;
    temp_pose.x = x;
    temp_pose.y = y;
    temp_pose.z = z;
    temp_pose.cam_angle = cam_angle;
    temp_pose.paw_angle = paw_angle;
    arm_control.push_back(temp_pose);
    return arm_control.size() - 1; // 返回新加入元素的索引值
}

int RobotArm::add_arm_pose_from_define(arm_pose pose)
{
    arm_control.push_back(pose);
    return arm_control.size() - 1; // 返回新加入元素的索引值
}


void RobotArm::arm_pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
nav_msgs::Odometry target_arm_pose_sub;
target_arm_pose_sub = *msg;
current_arm_pose.cam_angle = target_arm_pose_sub.pose.pose.position.x;
current_arm_pose.x = target_arm_pose_sub.pose.pose.orientation.x;
current_arm_pose.y = target_arm_pose_sub.pose.pose.orientation.y;
current_arm_pose.z = target_arm_pose_sub.pose.pose.orientation.z;
current_arm_pose.paw_angle = target_arm_pose_sub.pose.pose.orientation.w;
//ROS_INFO("x=%.2f,y=%.2f,z=%.2f",cur_pose.pose.pose.position.x,cur_pose.pose.pose.position.y,cur_pose.pose.pose.position.z);
}

void RobotArm::vision_cb(const std_msgs::ColorRGBA::ConstPtr& msg)
{
std_msgs::ColorRGBA temp_vision_msg;
temp_vision_msg = *msg;
temp_vision_msg.r = vision_data_.x;
temp_vision_msg.g = vision_data_.y;
temp_vision_msg.b = vision_data_.color;
//ROS_INFO("x=%.2f,y=%.2f,z=%.2f",cur_pose.pose.pose.position.x,cur_pose.pose.pose.position.y,cur_pose.pose.pose.position.z);
}

void RobotArm::arm_pose_pub(int index){
        nav_msgs::Odometry target_arm_pose_pub;
        target_arm_pose_pub.pose.pose.position.x = arm_control[index].cam_angle; // ע������Ӧ���� num.x ������ arm_control[num].cam_angle
        target_arm_pose_pub.pose.pose.orientation.x = arm_control[index].x;
        target_arm_pose_pub.pose.pose.orientation.y = arm_control[index].y;
        target_arm_pose_pub.pose.pose.orientation.z = arm_control[index].z;
        target_arm_pose_pub.pose.pose.orientation.w = arm_control[index].paw_angle;
        arm_pub.publish(target_arm_pose_pub); // ����Ϊ target_arm_pose_pub
        ROS_INFO("arm_moving_to_target");
        ros::Duration(0.1).sleep();
}





bool RobotArm::arm_arrived(arm_pose target_arm_pose){
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

    



