#include "thread_initial_unit.hpp"
thread_initial_unit::thread_initial_unit(){

}
thread_initial_unit::thread_initial_unit(thread_terminal_unit* t_terminal){
    thread_terminals.push_back(t_terminal);
}

void thread_initial_unit::runTasks(){
    if (thread_terminals[0]!=NULL){
        thread_terminals[0]->lock_mutex(); //TODO check if this can be moved to thread_terminal_unit
        DataMessage* current_msg=thread_terminals[0]->clone_last_message();
        thread_terminals[0]->unlock_mutex();
        this->emit_message(current_msg);  
        delete current_msg;
    }
}

void thread_initial_unit::addTerminalUnit(thread_terminal_unit* t_terminal){
    thread_terminals.push_back(t_terminal);
}
