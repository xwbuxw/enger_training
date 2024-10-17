#include "state_machine_.h"
#include <ros/ros.h>



void cur_pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    cur_pose = *msg;
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    ros::Rate rate(1);
    RobotFSM fsm;
    ros::Subscriber curr_pose_sub = nh.subscribe<nav_msgs::Odometry>
        ("/aft_mapped_to_init", 10, cur_pose_cb);
    ros::Publisher tar_pose_pub = nh.advertise<nav_msgs::Odometry>("/target_pose", 10);
    target_pose.pose.pose.position.x=0;
    while(ros::ok()){
        fsm.processEvent(Event::TEST);
        tar_pose_pub.publish(target_pose);
        //fsm.processEvent(Event::QR_CODE_READ);
        // fsm.processEvent(Event::BATCH_FETCHED);
        // fsm.processEvent(Event::BATCH_DELIVERED);
        // fsm.processEvent(Event::BATCH_STORED);
        // fsm.processEvent(Event::BATCH_FETCHED);
        // fsm.processEvent(Event::BATCH_DELIVERED);
        // fsm.processEvent(Event::BATCH_STORED);
        // fsm.processEvent(Event::RETURNED_TO_START);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
