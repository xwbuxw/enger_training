#ifndef STATE_MACHINE__H
#define STATE_MACHINE__H

#include <ros/ros.h>

enum class State {
    INIT,
    READ_QR_CODE,
    FETCH_FIRST_BATCH,
    DELIVER_TO_PROCESSING,
    STORE_FIRST_BATCH,
    FETCH_SECOND_BATCH,
    STORE_SECOND_BATCH,
    RETURN_TO_START,
    COMPLETE,
    TEST
};

enum class Event {
    QR_CODE_READ,
    BATCH_FETCHED,
    BATCH_DELIVERED,
    BATCH_STORED,
    RETURNED_TO_START,
    TASK_COMPLETE,
    TEST
};

class RobotFSM {
public:
    RobotFSM() : currentState(State::INIT) {}

    void processEvent(Event event);
    
private:
    int fechting_order;

    ros::NodeHandle nh_;

    State currentState;

    void handleInit(Event event);

    void handleReadQRCode(Event event);

    void handleFetchFirstBatch(Event event);

    void handleDeliverToProcessing(Event event);

    void handleStoreFirstBatch(Event event);

    void handleFetchSecondBatch(Event event);

    void handleStoreSecondBatch(Event event);

    void handleReturnToStart(Event event);

    void handleComplete(Event event);
};

#endif
