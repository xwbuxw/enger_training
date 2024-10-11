#include <iostream>
#include <state_machine_.h>


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

void RobotFSM::handleInit(Event event){
    if (event == Event::QR_CODE_READ) {
        currentState = State::READ_QR_CODE;
        std::cout << "Reading QR code." << std::endl;
    }
}

void RobotFSM::handleReadQRCode(Event event) {
    if (event == Event::QR_CODE_READ) {
        currentState = State::FETCH_FIRST_BATCH;
        std::cout << "Fetching first batch of materials." << std::endl;
    }
}

void RobotFSM::handleFetchFirstBatch(Event event) {
    if (event == Event::BATCH_FETCHED) {
        currentState = State::DELIVER_TO_PROCESSING;
        std::cout << "Delivering first batch to processing area." << std::endl;
    }
}

void RobotFSM::handleDeliverToProcessing(Event event) {
    if (event == Event::BATCH_DELIVERED) {
        currentState = State::STORE_FIRST_BATCH;
        std::cout << "Storing first batch in temporary storage." << std::endl;
    }
}

void RobotFSM::handleStoreFirstBatch(Event event) {
    if (event == Event::BATCH_STORED) {
        currentState = State::FETCH_SECOND_BATCH;
        std::cout << "Fetching second batch of materials." << std::endl;
    }
}

void RobotFSM::handleFetchSecondBatch(Event event) {
    if (event == Event::BATCH_FETCHED) {
        //currentState = State::DELIVER_TO_PROCESSING;
        std::cout << "Delivering second batch to processing area." << std::endl;
    }
}

void RobotFSM::handleStoreSecondBatch(Event event) {
    if (event == Event::BATCH_STORED) {
        //currentState = State::RETURN_TO_START;
        std::cout << "Returning to start." << std::endl;
    }
}

void RobotFSM::handleReturnToStart(Event event) {
    if (event == Event::RETURNED_TO_START) {
        //currentState = State::COMPLETE;
        std::cout << "Task complete." << std::endl;
    }
}

void RobotFSM::handleComplete(Event event) {
    // Task is complete, no further action needed
    std::cout << "Task completed." << std::endl;
}

