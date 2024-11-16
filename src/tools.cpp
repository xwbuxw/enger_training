#include "tools.h"
#include "ros/console.h"
#include <cmath>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include "state_machine_.h"


std::vector<set_target_poses> set_tar_poses;
bool cur_pose_is_ok = false;
bool yaw_is_ok = false;


nav_msgs::Odometry trans_global2car(const nav_msgs::Odometry& target_pose, const nav_msgs::Odometry& cur_pose, double target_yaw) {
    // 提取 cur_pose 中的旋转信息并生成 tf2 Transform
    tf2::Transform cur_transform;
    tf2::Quaternion cur_orientation;
    tf2::fromMsg(cur_pose.pose.pose.orientation, cur_orientation);
    cur_orientation.normalize();
    cur_transform.setRotation(cur_orientation);
    cur_transform.setOrigin(tf2::Vector3(cur_pose.pose.pose.position.x, cur_pose.pose.pose.position.y, cur_pose.pose.pose.position.z));

    // 验证 cur_orientation
if (!std::isfinite(cur_orientation.x()) || !std::isfinite(cur_orientation.y()) || !std::isfinite(cur_orientation.z()) || !std::isfinite(cur_orientation.w())) {
    ROS_ERROR("cur_orientation is usaless");
} else {
    cur_pose_is_ok = 1;
}

    // 提取 target_pose 的位置并生成 tf2 Vector3
    tf2::Vector3 target_position(target_pose.pose.pose.position.x, target_pose.pose.pose.position.y, target_pose.pose.pose.position.z);

    // 计算相对于 cur_pose 的坐标
    tf2::Vector3 transformed_position = cur_transform.inverse() * target_position;

    // 构建返回的 nav_msgs::Odometry 对象，只设置位置部分
    nav_msgs::Odometry transformed_odom;
    transformed_odom.pose.pose.position.x = transformed_position.x();
    transformed_odom.pose.pose.position.y = transformed_position.y();
    transformed_odom.pose.pose.position.z = transformed_position.z();

    // 在逆运算和乘法之后，检查 transformed_position
    if (!std::isfinite(transformed_position.x()) || !std::isfinite(transformed_position.y()) || !std::isfinite(transformed_position.z())) {
        ROS_ERROR("transformed_position is usaless");
    }

    // 计算目标偏航角并归一化至 [-π, π]
    double tar_yaw = target_yaw - tf::getYaw(cur_pose.pose.pose.orientation)  ;
    while (tar_yaw < -M_PI) tar_yaw += 2 * M_PI;
    while (tar_yaw > M_PI) tar_yaw -= 2 * M_PI;
    if(tar_yaw < 0.03) {
        yaw_is_ok = true;
    }
    // 构造目标姿态的四元数并赋值
    tf2::Quaternion target_q;
    target_q.setRPY(0, 0, tar_yaw);
    target_q.normalize();
    transformed_odom.pose.pose.orientation = tf2::toMsg(target_q);
    

    ROS_INFO("Transformed target position: (%.2f, %.2f, %.2f)", transformed_odom.pose.pose.position.x, transformed_odom.pose.pose.position.y, transformed_odom.pose.pose.position.z);
    return transformed_odom;
}

void add_tar_pose(float x, float y, float yaw) {
    set_target_poses add_tar_pose_pose;
    add_tar_pose_pose.x = x;
    add_tar_pose_pose.y = y;
    add_tar_pose_pose.yaw = yaw;

    
    set_tar_poses.push_back(add_tar_pose_pose);
    ROS_INFO("Added waypoint: (%.2f, %.2f, %.2f)", x, y, yaw);
}