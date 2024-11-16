#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>

// 全局发布器
ros::Publisher car_pose_puber;

// 偏移量（根据实际情况修改）
const double x_offset = -0.3; // 雷达到车身中心的 x 偏移
const double y_offset = -0.2; // 雷达到车身中心的 y 偏移
const double z_offset = 0.0;  // 雷达到车身中心的 z 偏移

// 回调函数：处理雷达坐标系下的 Odometry
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // 创建 TF 变换
    tf2::Transform laser_to_base;
    laser_to_base.setOrigin(tf2::Vector3(x_offset, y_offset, z_offset));
    laser_to_base.setRotation(tf2::Quaternion(0, 0, 0, 1)); // 无额外旋转

    // 从输入的 Odometry 提取位置和方向
    tf2::Transform laser_pose;
    tf2::fromMsg(msg->pose.pose, laser_pose);

    // 执行坐标系变换
    tf2::Transform base_pose = laser_to_base * laser_pose;

    
    geometry_msgs::Odometry base_pose_msg;
    base_pose_msg = tf2::toMsg(base_pose);

    // 转换回 ROS 消息类型
    nav_msgs::Odometry transformed_odom = *msg; // 复制原消息
    transformed_odom.header.frame_id = "base_link"; // 更新坐标系
    transformed_odom.pose.pose = base_pose_msg.pose.pose;

    // 发布转换后的数据
    car_pose_puber.publish(transformed_odom);
}

int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "laser_to_base_transformer");
    ros::NodeHandle nh;

    // 订阅雷达的 Odometry 数据
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("laser_odom", 10, odomCallback);

    // 发布转换后的 Odometry 数据
    car_pose_puber = nh.advertise<nav_msgs::Odometry>("car_pose", 10);

    ROS_INFO("Laser to Base Transformer Node started.");
    ros::spin();

    return 0;
}
