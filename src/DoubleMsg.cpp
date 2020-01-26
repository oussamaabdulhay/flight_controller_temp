#include "DoubleMsg.hpp"

DoubleMsg::DoubleMsg()
{
}

DoubleMsg::~DoubleMsg()
{

}

msg_type DoubleMsg::getType()
{
    return msg_type::DOUBLE;
}

const int DoubleMsg::getSize()
{
    return sizeof(DoubleMsg);
}