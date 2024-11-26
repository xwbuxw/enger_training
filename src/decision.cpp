#include "ros/console.h"
#include "state_machine_.h"
#include <ros/ros.h>
#include "tools.h"


// nav_msgs::Odometry cur_pose;
// nav_msgs::Odometry target_pose;

//校赛逻辑：event=BATCH_STORED(处理放置)
//state = INIT->DELIVER_TO_PROCESSING->STORE_FIRST_BATCH->COMPLETE
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
    fsm.set_state(State::INIT);
    while(ros::ok()){
        fsm.processEvent(Event::BATCH_DELIVERED);
        tar_pose_pub.publish(pub_target_pose);
        ros::spinOnce();
        rate.sleep();
    }

    for(int i=5;i>0;i--){
        pub_target_pose = cur_pose;
        tar_pose_pub.publish(pub_target_pose); 
    }
    return 0;
}
