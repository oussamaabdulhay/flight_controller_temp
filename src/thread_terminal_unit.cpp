#include "thread_terminal_unit.hpp"
using namespace std;

void thread_terminal_unit::receive_msg_data(DataMessage* t_msg){
    this->lock_mutex();
    synced_message[id].push(t_msg->Clone());
    this->unlock_mutex();
}

DataMessage* thread_terminal_unit::clone_last_message(){
    synced_message[id].pop();
    if(synced_message[id].size() < 1){ //This is to handle the beggining of the code. It doesn't happen later.
        synced_message[id].push(new XSensMessage);
        std::cout << "WARNING" << std::endl;
    }
    return synced_message[id].front();
}

thread_terminal_unit::thread_terminal_unit(){
    id=thread_terminal_counter;
    thread_terminal_counter++;
    synced_message[id].push(new XSensMessage());//This is dummy to prevent possible segmentation fault at first read
}

void thread_terminal_unit::lock_mutex(){
    sync_lock[id].lock();
}
void thread_terminal_unit::unlock_mutex(){
    sync_lock[id].unlock();
}