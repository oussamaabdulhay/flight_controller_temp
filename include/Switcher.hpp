#pragma once
#include "Block.hpp"
#include <list>
#include "MsgReceiver.hpp"
#include "MsgEmitter.hpp"
#include "Vector3DMessage.hpp"
#include "PIDController.hpp"
#include "ProcessVariableReference.hpp"
#include <algorithm>
#include "logger.hpp"
#include "SwitchBlockMsg.hpp"
#include "ControlSystem.hpp"

class Switcher : public msg_receiver, public msg_emitter{

    private:
        std::list<Block*> _blocks;
        std::list<Block*>::iterator _it;
        switcher_type _type;
        Block* _active_block;

    public:
        enum unicast_addresses {broadcast, unicast_controller_switcher, unicast_control_system};
        enum receiving_channels {ch_broadcast, ch_provider, ch_error, ch_reference};
        void addBlock(Block* b);
        switcher_type getType();
        Block* getActiveBlock();
        void receive_msg_data(DataMessage* t_msg);
        void receive_msg_data(DataMessage* t_msg, int t_channel);
        Switcher(switcher_type t_type);
        ~Switcher();
};