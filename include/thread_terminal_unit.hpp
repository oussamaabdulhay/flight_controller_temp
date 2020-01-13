#pragma once

#include "MsgReceiver.hpp"
#include <boost/thread.hpp>
#include "XSensMessage.hpp"
#include <queue>
#include <deque>
#include <iostream>

//TODO Move this to a different header and source files
template <typename T, int MaxLen, typename Container=std::deque<T>>
class FixedQueue : public std::queue<T, Container> {
public:
    void push(const T& value) {
        if (this->size() == MaxLen) {
           this->c.pop_front();
        }
        std::queue<T, Container>::push(value);
    }
};

const int max_num_of_thread_terminals=100;
static boost::mutex sync_lock[max_num_of_thread_terminals];
static DataMessage* synced_message[max_num_of_thread_terminals];
static int thread_terminal_counter=0;
class thread_terminal_unit : public msg_receiver {
private:
    int id;
    
public:
    thread_terminal_unit();
    void receive_msg_data(DataMessage* t_msg) override; 
    DataMessage* clone_last_message();
    void lock_mutex();
    void unlock_mutex();
};