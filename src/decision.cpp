#include "state_machine_.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    ros::Rate rate(1);
    RobotFSM fsm;
    while(ros::ok()){
        fsm.processEvent(Event::TEST);
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
