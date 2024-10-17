#include "ros/console.h"
#include "ros/publisher.h"
#include <cmath>
#include <iostream>
#include <state_machine_.h>
#include <ros/ros.h>



void RobotFSM::processEvent(Event event) {
    switch (currentState) {
        case State::INIT:
            handleInit(event);
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

void RobotFSM::handleInit(Event event){//在这里加入一键启动代码，现在模拟一键启动
    fechting_order = 0;
    if (event == Event::TEST) {
        currentState = State::TEST;
        ROS_INFO("TEST_START!");
    }
    if(target_pose.pose.pose.position.x==0){
        target_pose = cur_pose;
        target_pose.pose.pose.position.x+=0.2; 
    }
    double x_error = target_pose.pose.pose.position.x - cur_pose.pose.pose.position.x;
    if(fabs(x_error)<0.05){
        ROS_INFO("arrived!");
        currentState = State::COMPLETE;
    }
    // if (event == Event::QR_CODE_READ) {
    //     currentState = State::READ_QR_CODE;
    //     ROS_INFO("Read QR code.");
    // }
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
    target_pose = cur_pose;
    ROS_INFO("Task completed.");
}

