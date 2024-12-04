#include "ros/console.h"
#include "ros/init.h"
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
#include "robo_arm.h"

nav_msgs::Odometry cur_pose;
nav_msgs::Odometry pub_target_pose;
nav_msgs::Odometry set_target_pose;
int set_tar_pose_index = 0;
float set_tar_yaw = 0;
float eg = 0;
bool add_flag = 0;
int move_index;


void move(int index){
    
    
        set_target_pose.pose.pose.position.x = set_tar_poses[index].x;
        set_target_pose.pose.pose.position.y = set_tar_poses[index].y;
        ROS_INFO("set_target_pose.pose.pose.position: (%.2f, %.2f)", set_target_pose.pose.pose.position.x, set_target_pose.pose.pose.position.y);

        set_tar_yaw = set_tar_poses[index].yaw;
        ROS_INFO("currx is %.5f",cur_pose.pose.pose.position.x);
        pub_target_pose = trans_global2car( set_target_pose, cur_pose, set_tar_yaw);
        
        //ROS_INFO("pubtarx: (%.2f)", pub_target_pose.pose.pose.position.x);
        
        //target_pose.pose.pose.position.x-=0.5;
        
        //ROS_INFO("x=%.2f,y=%.2f,z=%.2f",cur_pose.pose.pose.position.x,cur_pose.pose.pose.position.y,cur_pose.pose.pose.position.z);
        // double x_error = set_target_pose.pose.pose.position.x - cur_pose.pose.pose.position.x;
        // double y_error = set_target_pose.pose.pose.position.y - cur_pose.pose.pose.position.y;
        // //ROS_INFO("error_y:%d",cur_pose_is_ok);
        // ROS_INFO("index:%d",index); 
        // if(cur_pose_is_ok == 1){
        //     if(fabs(x_error)<0.05 && fabs(y_error)<0.05 && yaw_is_ok == 1){
        //         yaw_is_ok = 0;
        //         ROS_INFO("arrived!");
        //         if(index < set_tar_poses.size() - 1) {
        //             index ++;
        //         }
        //     //currentState = State::COMPLETE;
        //     }
        // ROS_INFO("x=%.2f,y=%.2f,z=%.2f",
        // target_pose.pose.pose.position.x,target_pose.pose.pose.position.y,target_pose.pose.pose.position.z);
        //target_pose.pose.pose.orientation = cur_pose.pose.pose.orientation;
        // }
   
}

bool car_arrive (int index) {
    nav_msgs::Odometry temp_pose;
    temp_pose.pose.pose.position.x = set_tar_poses[index].x;
    temp_pose.pose.pose.position.y = set_tar_poses[index].y;
    double x_error = temp_pose.pose.pose.position.x - cur_pose.pose.pose.position.x;
    double y_error = temp_pose.pose.pose.position.y - cur_pose.pose.pose.position.y;
    if(fabs(x_error)<0.05 && fabs(y_error)<0.05 && yaw_is_ok == 1){
                yaw_is_ok = 0;
                ROS_INFO("arrived!");
                return 1;
            //currentState = State::COMPLETE;
    } else return 0;
}


void RobotFSM::processEvent(Event event) {
    nav_msgs::Odometry return_value;
    switch (currentState) {
        case State::INIT:
            handleInit(event);//首先执行这个，用于检查机器状态
            break;
        case State::READ_QR_CODE:
            handleReadQRCode(event);
            break;
        case State::FETCH_FIRST_BATCH:
            handleFetchFirstBatch(event);
            break;
        case State::DELIVER_TO_PROCESSING:
            handleDeliverToProcessing(event);//然后执行这个，开到粗加工区
            break;
        case State::STORE_FIRST_BATCH:
            handleStoreFirstBatch(event);//最后执行这个，放置物体
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
        case State::TEST:
            handleTest(event);
            break;
        case State::TEST_VISION:
            handleTest_vision(event);
            break;
        }
}

// void RobotFSM::processEvent(Event event) {
//     handleInit(event);
//     ROS_INFO("process_init");
// }

void RobotFSM::handleTest(Event event){
    robot_arm_.test();
}

void RobotFSM::handleTest_vision(Event event){
    
}

void RobotFSM::handleInit(Event event){
    ROS_INFO("MISSION_START!");
    //while(cur_pose.pose.pose.position.x == 0);
    //{
        ROS_INFO("Waiting for SLAM to initialize...");
    //}
    ROS_INFO("Slam init finished!");//雷达完成初始化
    ROS_INFO("Init finished!");
    //TODO检查各个模块，还缺少检查地盘和检查机械臂的代码，需要标志位的检查
    currentState = State::DELIVER_TO_PROCESSING;
    //currentState = State::STORE_FIRST_BATCH;
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

void RobotFSM::handleDeliverToProcessing(Event event) { //步骤2，移动到粗加工区

    if (add_flag != 1) {
    move_index = add_tar_pose(0.2, 0, 0);
    add_tar_pose(0.2, 0.2, 0);
    add_tar_pose(0.2, 0.2, 0.2);
    add_flag = 1;
    }
    //add_tar_pose(0.2,0.2, 3.1415);//TODO 输入正确的坐标
    move(move_index);
    if (car_arrive(move_index)) {
        move_index ++;
    } 
    if(move_index = arm_control.size()) {
        add_flag = 1;
        currentState = State::STORE_FIRST_BATCH;
    }
    
    
    //ROS_INFO("Moved to processing area.");
    //currentState = State::STORE_FIRST_BATCH;
}

void RobotFSM::handleStoreFirstBatch(Event event) {
    // if (event == Event::BATCH_STORED) {
        
        robot_arm_.choose('r');
        robot_arm_.put_down('r');
        robot_arm_.choose('g');
        robot_arm_.put_down('g');
        robot_arm_.choose('b');
        robot_arm_.put_down('b');
        //int temp_index = 0;
        //temp_index = add_tar_pose(2, 2, 3.1415);//红原前面的坐标
        //move(temp_index);
        //robot_arm_.choose('b');
        //robot_arm_.put_down('g''r');
        //temp_index = add_tar_pose(2, 2, 3.1415);//蓝原前面的坐标
        //move(temp_index);
        //robot_arm_.choose('g');
        //robot_arm_.put_down('b''r');
        //temp_index = add_tar_pose(2, 2, 3.1415);//绿原前面的坐标
        //move(temp_index);
        

        currentState = State::FETCH_SECOND_BATCH;
        ROS_INFO("Fetching second batch of materials.");
    // }
    // currentState = State::COMPLETE;
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


