#ifndef TOOLS__H
#define TOOLS__H

#include <ros/ros.h>
#include "ros/console.h"
#include "ros/publisher.h"
#include <nav_msgs/Odometry.h>

struct set_target_poses {
    double x;
    double y;
    double yaw;
};
static std::vector<set_target_poses> set_tar_poses;



nav_msgs::Odometry trans_global2car(const nav_msgs::Odometry& target_pose, const nav_msgs::Odometry& cur_pose, double target_yaw);

#endif


