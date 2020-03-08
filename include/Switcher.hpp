#pragma once
#include "Block.hpp"
#include <list>
#include "MsgReceiver.hpp"
#include "MsgEmitter.hpp"
#include "ControlSystemMessage.hpp"
#include "SwitcherMessage.hpp"
#include "Vector3DMessage.hpp"
#include "PIDController.hpp"
#include "ProcessVariableReference.hpp"
#include <algorithm>
#include "logger.hpp"
#include "SwitchBlockMsg.hpp"

class Switcher : public msg_receiver, public msg_emitter{

    private:
        std::list<Block*> _blocks;
        std::list<Block*>::iterator _it;
        switcher_type _type;
        Block* _active_block;
        Vector3DMessage m_process_variable;
        SwitcherMessage m_reference_msg;
        SwitcherMessage m_out_switcher_msg;

    public:
        void addBlock(Block* b);
        switcher_type getType();
        Block* getActiveBlock();
        void receive_msg_data(DataMessage* t_msg);
        void loopInternal();
        //TODO Send a message to Block
        //TODO Receive a message from Block
        Switcher(switcher_type t_type);
        ~Switcher();
};