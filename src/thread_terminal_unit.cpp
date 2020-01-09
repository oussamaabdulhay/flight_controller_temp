#include "thread_terminal_unit.hpp"
using namespace std;

void thread_terminal_unit::receive_msg_data(DataMessage* t_msg){
    this->lock_mutex();
    synced_message[id]=t_msg->Clone();
    this->unlock_mutex();
}

DataMessage* thread_terminal_unit::clone_last_message(){
    return synced_message[id]->Clone();
}

thread_terminal_unit::thread_terminal_unit(){
    id=thread_terminal_counter;
    thread_terminal_counter++;
    synced_message[id]=new XSensMessage();
}

void thread_terminal_unit::lock_mutex(){
    sync_lock[id].lock();
}
void thread_terminal_unit::unlock_mutex(){
    sync_lock[id].unlock();
}