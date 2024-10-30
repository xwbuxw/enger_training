#include "ros/console.h"
#include "state_machine_.h"
#include <ros/ros.h>


// nav_msgs::Odometry cur_pose;
// nav_msgs::Odometry target_pose;
void cur_pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    cur_pose = *msg;
    //ROS_INFO("x=%.2f,y=%.2f,z=%.2f",cur_pose.pose.pose.position.x,cur_pose.pose.pose.position.y,cur_pose.pose.pose.position.z);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    ros::Rate rate(50);
    RobotFSM fsm;
    ros::Subscriber curr_pose_sub = nh.subscribe<nav_msgs::Odometry>
        ("/aft_mapped_to_init", 10, cur_pose_cb);
    ros::Publisher tar_pose_pub = nh.advertise<nav_msgs::Odometry>("/target_pose", 10);
    set_target_pose.pose.pose.position.x=0;
    fsm.set_state(State::TEST);
    while(ros::ok()){
        fsm.processEvent(Event::TEST);
        tar_pose_pub.publish(pub_target_pose);
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
