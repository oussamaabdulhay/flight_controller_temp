#include "BooleanMsg.hpp"

BooleanMsg::BooleanMsg() {

}

BooleanMsg::~BooleanMsg() {

}

msg_type BooleanMsg::getType()
{
    return msg_type::BOOLEAN;
}

const int BooleanMsg::getSize()
{
    return sizeof(BooleanMsg);
}
