#include "thread_initial_unit.hpp"
thread_initial_unit::thread_initial_unit(){

}
thread_initial_unit::thread_initial_unit(thread_terminal_unit* t_terminal){
    thread_terminal = t_terminal;
}

void thread_initial_unit::runTasks(){
    if (thread_terminal!=NULL){
        thread_terminal->lock_mutex();
        DataMessage* current_msg=thread_terminal->clone_last_message();
        thread_terminal->unlock_mutex();
        this->emit_message(current_msg);  
        delete current_msg;
    }
}