#pragma once
#include "TimedBlock.hpp"
#include "thread_terminal_unit.hpp"
#include "MsgEmitter.hpp"

class thread_initial_unit : public TimedBlock, public msg_emitter {
private:
    thread_terminal_unit* thread_terminal;
public:
    void runTasks(); 
    void loopInternal() {};

    thread_initial_unit();
    thread_initial_unit(thread_terminal_unit*);
};