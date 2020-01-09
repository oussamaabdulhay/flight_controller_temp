#include "looper.hpp"

Looper* Looper::_instance_ptr = NULL;
pthread_mutex_t Looper::lock;

Looper::Looper(){
    _instance_ptr = this;
    if (pthread_mutex_init(&lock, NULL) != 0) { 
        Logger::getAssignedLogger()->log("Mutex Failed to Create ",LoggerLevel::Error);
        //return 1; 
    }
}

void* Looper::Loop1KHz(void *vargp) 
{ 
    Timer* _loop_timer = new Timer();
    
    int j = 0;
    int p = 0;

    while(1){
        _loop_timer->tick();
        pthread_mutex_lock(&lock); 
        for (TimedBlock* const& i : _instance_ptr->_timed_blocks){
            if(i->getLoopTime() == block_frequency::hz1000){
                i->loopInternal();
                j++;
            }
        }
        pthread_mutex_unlock(&lock); 
        int consumed_time =_loop_timer->tockMicroSeconds();
        //std::cout << "Consumed Time 1kHz: " << consumed_time << std::endl;
        if (consumed_time>Loop1KHz_dt){
            //Logger::getAssignedLogger()->log("exceeded loop time 1khz ",LoggerLevel::Warning);
            //std::cout << "Consumed Time 1kHz" << consumed_time << std::endl;
            std::cout << "Exceeded 400Hz. Consumed Time:  " << consumed_time << " on the " << j++ << " loop." << std::endl;
        }
        else{
            //std::cout << "Sleep Time" << Loop1KHz_dt-consumed_time << std::endl;
            usleep(Loop1KHz_dt-consumed_time);
        }
    }
} 

void* Looper::Loop200Hz(void *vargp) 
{ 
    Timer* _loop_timer = new Timer();
    
    int j = 0;
    int p = 0;

    while(1){
        _loop_timer->tick();
        pthread_mutex_lock(&lock); 
        for (TimedBlock* const& i : _instance_ptr->_timed_blocks){
            if(i->getLoopTime() == block_frequency::hz200){
                i->loopInternal();
                j++;
            }
        }
        pthread_mutex_unlock(&lock); 
        int consumed_time =_loop_timer->tockMicroSeconds();
        //std::cout << "Consumed Time 1kHz: " << consumed_time << std::endl;
        if (consumed_time>Loop200Hz_dt){
            //Logger::getAssignedLogger()->log("exceeded loop time 1khz ",LoggerLevel::Warning);
            //std::cout << "Consumed Time 1kHz" << consumed_time << std::endl;
            std::cout << "Exceeded 200Hz. Consumed Time:  " << consumed_time << " on the " << j++ << " loop." << std::endl;
        }
        else{
            //std::cout << "Sleep Time" << Loop200Hz_dt-consumed_time << std::endl;
            usleep(Loop200Hz_dt-consumed_time);
        }
    }
} 

void* Looper::Loop400Hz(void *vargp) 
{ 
    Timer* _loop_timer = new Timer();
    
    int j = 0;
    int p = 0;

    while(1){
        _loop_timer->tick();
        pthread_mutex_lock(&lock); 
        for (TimedBlock* const& i : _instance_ptr->_timed_blocks){
            if(i->getLoopTime() == block_frequency::hz400){
                i->loopInternal();
                j++;
            }
        }
        pthread_mutex_unlock(&lock); 
        int consumed_time =_loop_timer->tockMicroSeconds();
        //std::cout << "Consumed Time 1kHz: " << consumed_time << std::endl;
        if (consumed_time>Loop400Hz_dt){
            //Logger::getAssignedLogger()->log("exceeded loop time 1khz ",LoggerLevel::Warning);
            //std::cout << "Consumed Time 1kHz" << consumed_time << std::endl;
            std::cout << "Exceeded 400Hz. Consumed Time:  " << consumed_time << " on the " << j++ << " loop." << std::endl;
        }
        else{
            //std::cout << "Sleep Time" << Loop200Hz_dt-consumed_time << std::endl;
            usleep(Loop400Hz_dt-consumed_time);
        }
    }
} 

void* Looper::hardwareLoop1KHz(void *vargp) 
{ 
    Timer* _loop_timer = new Timer();
    
    int j = 0;
    int p = 0;

    while(1){
        _loop_timer->tick();
        pthread_mutex_lock(&lock); 
        for (TimedBlock* const& i : _instance_ptr->_timed_blocks){
            if(i->getLoopTime() == block_frequency::hhz200){
                i->loopInternal();
                j++;
            }
        }
        pthread_mutex_unlock(&lock); 
        int consumed_time =_loop_timer->tockMicroSeconds();
        //std::cout << "Consumed Time Hardware 1kHz: " << consumed_time << std::endl;
        if (consumed_time>hardwareLoop1KHz_dt){
            //Logger::getAssignedLogger()->log("exceeded loop time hardware 1khz ",LoggerLevel::Warning);
            std::cout << "Consumed Time hard1kHZ" << consumed_time << std::endl;
            //std::cout << "Exceeded hard1kHZ: " << j++ << std::endl;

        }
        else{
            usleep(hardwareLoop1KHz_dt-consumed_time);
        }
    }
} 

void* Looper::Loop100Hz(void *vargp) 
{ 
    Timer* _loop_timer = new Timer();
    while(1){
        _loop_timer->tick();
        pthread_mutex_lock(&lock); 
        for (TimedBlock* const& j : _instance_ptr->_timed_blocks){
            if(j->getLoopTime() == block_frequency::hz100){
                j->loopInternal();
            }
        }
        pthread_mutex_unlock(&lock); 
        int consumed_time =_loop_timer->tockMicroSeconds();
        //std::cout << "Consumed Time 100Hz: " << consumed_time << std::endl;
        if (consumed_time>Loop100Hz_dt){
            //Logger::getAssignedLogger()->log("exceeded loop time 100hz",LoggerLevel::Warning);
        }
        else{
            usleep(Loop100Hz_dt-consumed_time);
        }
    }
} 

void* Looper::Loop120Hz(void *vargp) 
{ 
    Timer* _loop_timer = new Timer();
    while(1){
        _loop_timer->tick();
        pthread_mutex_lock(&lock); 
        for (TimedBlock* const& j : _instance_ptr->_timed_blocks){
            if(j->getLoopTime() == block_frequency::hz120){
                j->loopInternal();
            }
        }
        pthread_mutex_unlock(&lock); 
        int consumed_time =_loop_timer->tockMicroSeconds();
        //std::cout << "Consumed Time 120Hz: " << consumed_time << std::endl;
        if (consumed_time>Loop120Hz_dt){
            //Logger::getAssignedLogger()->log("exceeded loop time 120hz",LoggerLevel::Warning);
        }
        else{
            usleep(Loop120Hz_dt-consumed_time);
        }
    }
} 

void Looper::addTimedBlock(TimedBlock* t_cs){
    _timed_blocks.push_back(t_cs);
}
