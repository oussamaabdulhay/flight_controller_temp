#pragma once
#include "Switcher.hpp"
#include "MsgEmitter.hpp"
#include "MsgReceiver.hpp"
#include <vector>
#include "ControlSystemMessage.hpp"
#include "PID_values.hpp"
#include "UpdatePoseMessage.hpp"
#include "TimedBlock.hpp"
#include "ROSMsg.hpp"
#include "SwitchBlockMsg.hpp"
#include "Timer.hpp"
#include "DoubleMsg.hpp"
#include "ROSUnit_BroadcastData.hpp"
#include "VectorDoubleMsg.hpp"

class ControlSystem : public TimedBlock, public msg_emitter, public msg_receiver{

    private:
        Timer timer;
        control_system _control_system;
        Switcher* controllerSwitcher;
        Switcher* referenceSwitcher;
        std::vector<Switcher*> _switchers;
        block_frequency _frequency;
        float _dt;

    public:
        enum receiving_channels {ch_broadcast, ch_reference, ch_controller};
        enum unicast_addresses {broadcast, unicast_controller_switcher, unicast_reference_switcher, unicast_control_system};
        void receive_msg_data(DataMessage* t_msg);
        void receive_msg_data(DataMessage* t_msg, int t_channel);
        void getStatus();
        void addBlock(Block* t_block);
        control_system getControlSystemType();
        float get_dt() {return _dt;}

        ControlSystem(control_system, block_frequency);
        ~ControlSystem(); //TODO prevent automatic storage

    
};