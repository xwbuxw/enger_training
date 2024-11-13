#ifndef TOOLS__H
#define TOOLS__H

#include <ros/ros.h>
#include "ros/console.h"
#include "ros/publisher.h"
#include <nav_msgs/Odometry.h>

struct set_target_poses {
    float x;
    float y;
    float yaw;
};
extern std::vector<set_target_poses> set_tar_poses;



nav_msgs::Odometry trans_global2car(const nav_msgs::Odometry& target_pose, const nav_msgs::Odometry& cur_pose, double target_yaw);
void add_tar_pose(float x, float y, float yaw);


#endif


