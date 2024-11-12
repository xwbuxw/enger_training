#include "ros/console.h"
#include "ros/publisher.h"
#include <cmath>
#include <iostream>
#include <state_machine_.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tools.h"
#include <vector>


nav_msgs::Odometry cur_pose;
nav_msgs::Odometry pub_target_pose;
nav_msgs::Odometry set_target_pose;
int set_tar_pose_index = 0;

std::vector<std::tuple<int, int, int>> coordinates(4) ;
    

void RobotFSM::processEvent(Event event) {
    nav_msgs::Odometry return_value;
    switch (currentState) {
        case State::TEST:
            handleInit(event);
            //ROS_INFO("process_init");
            break;
        case State::READ_QR_CODE:
            handleReadQRCode(event);
            break;
        case State::FETCH_FIRST_BATCH:
            handleFetchFirstBatch(event);
            break;
        case State::DELIVER_TO_PROCESSING:
            handleDeliverToProcessing(event);
            break;
        case State::STORE_FIRST_BATCH:
            handleStoreFirstBatch(event);
            break;
        case State::FETCH_SECOND_BATCH:
            handleFetchSecondBatch(event);
            break;
        case State::STORE_SECOND_BATCH:
            handleStoreSecondBatch(event);
            break;
        case State::RETURN_TO_START:
            handleReturnToStart(event);
            break;
        case State::COMPLETE:
            handleComplete(event);
            break;
            
    }
}

// void RobotFSM::processEvent(Event event) {
//     handleInit(event);
//     ROS_INFO("process_init");
// }

void RobotFSM::handleInit(Event event){//在这里加入一键启动代码，现在模拟一键启动
    fechting_order = 0;
    if (event != Event::TEST) {
        currentState = State::TEST;
        ROS_INFO("TEST_START!");
    }
    
        
        set_target_pose.pose.pose.position.x = std::get<0>(coordinates[set_tar_pose_index]);
        set_target_pose.pose.pose.position.y = coordinates[set_tar_pose_index];
        pub_target_pose = trans_global2car( set_target_pose,cur_pose);
        
        //target_pose.pose.pose.position.x-=0.5;
    
    //ROS_INFO("x=%.2f,y=%.2f,z=%.2f",cur_pose.pose.pose.position.x,cur_pose.pose.pose.position.y,cur_pose.pose.pose.position.z);
    double x_error = set_target_pose.pose.pose.position.x - cur_pose.pose.pose.position.x;
    
    if(fabs(x_error)<0.05 ){
        ROS_INFO("arrived!");
        set_tar_pose_index ++;
        //currentState = State::COMPLETE;
    }
    // ROS_INFO("x=%.2f,y=%.2f,z=%.2f",
    // target_pose.pose.pose.position.x,target_pose.pose.pose.position.y,target_pose.pose.pose.position.z);
    //target_pose.pose.pose.orientation = cur_pose.pose.pose.orientation;
    
}

void RobotFSM::handleReadQRCode(Event event) {
    //fechting_order = get_QRCODE_order();
    nh_.setParam("fechting_order",fechting_order);
    //加入判断，如果参数服务器中的fechting_order为0，则重新执行扫码程序，超过五秒还没有扫描到，则发出警报，随后退出
    //如果参数服务器中的fechting_order不为0，则继续执行以下程序
    if (event == Event::QR_CODE_READ) {//在当前状态
        currentState = State::FETCH_FIRST_BATCH;
        ROS_INFO("Fetching first batch of materials.");
    }
}

void RobotFSM::handleFetchFirstBatch(Event event) {
    if (event == Event::BATCH_FETCHED) {
        currentState = State::DELIVER_TO_PROCESSING;
        ROS_INFO("Delivering first batch to processing area.");
    }
}

void RobotFSM::handleDeliverToProcessing(Event event) {
    if (event == Event::BATCH_DELIVERED) {
        currentState = State::STORE_FIRST_BATCH;
        ROS_INFO("Storing first batch in temporary storage.");
    }
}

void RobotFSM::handleStoreFirstBatch(Event event) {
    if (event == Event::BATCH_STORED) {
        currentState = State::FETCH_SECOND_BATCH;
        ROS_INFO("Fetching second batch of materials.");
    }
}

void RobotFSM::handleFetchSecondBatch(Event event) {
    if (event == Event::BATCH_FETCHED) {
        currentState = State::DELIVER_TO_PROCESSING;
        ROS_INFO("Delivering second batch to processing area.");
    }
}

void RobotFSM::handleStoreSecondBatch(Event event) {
    if (event == Event::BATCH_STORED) {
        currentState = State::RETURN_TO_START;
        ROS_INFO("Returning to start.");
    }
}

void RobotFSM::handleReturnToStart(Event event) {
    if (event == Event::RETURNED_TO_START) {
        currentState = State::COMPLETE;
        ROS_INFO("Task complete.");
    }
}

void RobotFSM::handleComplete(Event event) {
    // Task is complete, no further action needed
    // target_pose = cur_pose;
    ROS_INFO("Task completed.");
}

