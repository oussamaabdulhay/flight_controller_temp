#pragma once
#include "DataMessage.hpp"

class SwitchBlockMsg : public DataMessage {

private:
    msg_type _type;
    int _block_in, _block_out;

public:

    msg_type getType();
    const int getSize();
    void setSwitchBlockMsg(int, int);
    int getBlockToSwitchIn();
    int getBlockToSwitchOut();
    SwitchBlockMsg();
    ~SwitchBlockMsg();
};