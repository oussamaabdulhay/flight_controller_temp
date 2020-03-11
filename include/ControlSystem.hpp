#pragma once
#include "Switcher.hpp"
#include "MsgEmitter.hpp"
#include "MsgReceiver.hpp"
#include <vector>
#include "ControlSystemMessage.hpp"
#include "PID_values.hpp"
#include "TimedBlock.hpp"
#include "SwitchBlockMsg.hpp"
#include "Timer.hpp"
#include "DoubleMsg.hpp"
#include "ROSUnit_BroadcastData.hpp"
#include "VectorDoubleMsg.hpp"

class Switcher;
class ControlSystem : public TimedBlock, public MsgEmitter, public MsgReceiver{

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
        enum unicast_addresses {broadcast, unicast_controller_switcher, unicast_reference_switcher, unicast_control_system, unicast_actuation_system};
        void receiveMsgData(DataMessage* t_msg);
        void receiveMsgData(DataMessage* t_msg, int t_channel);
        void getStatus();
        void addBlock(Block* t_block);
        control_system getControlSystemType();
        float get_dt() {return _dt;}

        ControlSystem(control_system, block_frequency);
        ~ControlSystem(); //TODO prevent automatic storage

    
};