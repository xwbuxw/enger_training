#include <iostream>
#include <state_machine_.h>
int main() {
    RobotFSM fsm;
    
     fsm.processEvent(Event::QR_CODE_READ);
    // fsm.processEvent(Event::BATCH_FETCHED);
    // fsm.processEvent(Event::BATCH_DELIVERED);
    // fsm.processEvent(Event::BATCH_STORED);
    // fsm.processEvent(Event::BATCH_FETCHED);
    // fsm.processEvent(Event::BATCH_DELIVERED);
    // fsm.processEvent(Event::BATCH_STORED);
    // fsm.processEvent(Event::RETURNED_TO_START);

    return 0;
}