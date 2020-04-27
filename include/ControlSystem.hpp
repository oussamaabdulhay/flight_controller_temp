#pragma once
#include "Switcher.hpp"
#include "common_srv/MsgEmitter.hpp"
#include "common_srv/MsgReceiver.hpp"
#include <vector>
#include "ControlSystemMessage.hpp"
#include "PID_values.hpp"
#include "SwitchBlockMsg.hpp"
#include "common_srv/Timer.hpp"
#include "common_srv/PosesMsg.hpp"
#include "ROSUnit_BroadcastData.hpp"
#include "common_srv/VectorDoubleMsg.hpp"

class Switcher;
class ControlSystem : public MsgEmitter, public MsgReceiver{

    private:
        Timer timer;
        control_system _control_system;
        Switcher* controllerSwitcher;
        Switcher* referenceSwitcher;
        VectorDoubleMsg reference_ros_msg;
        VectorDoubleMsg controller_ros_msg;
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