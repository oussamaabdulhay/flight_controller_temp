#include "DoublePointerMsg.hpp"

DoublePointerMsg::DoublePointerMsg()
{
}

DoublePointerMsg::~DoublePointerMsg()
{

}

msg_type DoublePointerMsg::getType()
{
    return msg_type::DOUBLEPOINTER;
}

const int DoublePointerMsg::getSize()
{
    return sizeof(DoublePointerMsg);
}