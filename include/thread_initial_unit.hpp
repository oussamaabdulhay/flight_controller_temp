#pragma once
// Version: 1.1
// Author: M. Chehadeh
// Date: 2 Feb 2020
// Release note: Multiple terminal units can be added. runTasks overridable
#include "TimedBlock.hpp"
#include "thread_terminal_unit.hpp"
#include "MsgEmitter.hpp"
#include <vector>
#include "PVConcatenator.hpp"

class thread_initial_unit : public TimedBlock, public msg_emitter {
protected:
    std::vector<thread_terminal_unit*> thread_terminals;
public:
    virtual void runTasks(); 
    void loopInternal() {};
    void addTerminalUnit(thread_terminal_unit*);

    thread_initial_unit();
    thread_initial_unit(thread_terminal_unit*);
    
};