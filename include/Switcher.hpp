#pragma once
#include "Block.hpp"
#include <list>
#include "common_srv/MsgReceiver.hpp"
#include "common_srv/MsgEmitter.hpp"
#include "common_srv/Vector3DMessage.hpp"
#include "PIDController.hpp"
#include "ProcessVariableReference.hpp"
#include <algorithm>
#include "logger.hpp"
#include "SwitchBlockMsg.hpp"
#include "ControlSystem.hpp"

class Switcher : public MsgReceiver, public MsgEmitter{

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
        void receiveMsgData(DataMessage* t_msg);
        void receiveMsgData(DataMessage* t_msg, int t_channel);
        Switcher(switcher_type t_type);
        ~Switcher();
};