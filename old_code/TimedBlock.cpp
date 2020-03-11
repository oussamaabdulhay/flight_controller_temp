#include "TimedBlock.hpp"
#include <iostream>
TimedBlock::TimedBlock(block_frequency t_bf) {
    _frequency = t_bf;
}

TimedBlock::TimedBlock() {

}

TimedBlock::~TimedBlock() {

}

block_frequency TimedBlock::getLoopFrequency(){
    return _frequency;
}

void TimedBlock::tickTimer(){
    internal_timer.tick();
    this->runTasks();
}

bool TimedBlock::hasLoopTimeElapsed(){
    
    if (this->getLoopRemainingMicroSec()<=0){
        return true;
    }
    return false;
}
int TimedBlock::getLoopRemainingMicroSec(){
    return (this->getLoopPeriodMicroSec()-internal_timer.tockMicroSeconds());
}

int TimedBlock::getLoopPeriodMicroSec(){
    auto loop_period_us=1000000./((double)static_cast<int>(this->getLoopFrequency()));
    return loop_period_us;
}

void TimedBlock::setLoopFrequency(block_frequency t_block_freq){
    _frequency=t_block_freq;
}