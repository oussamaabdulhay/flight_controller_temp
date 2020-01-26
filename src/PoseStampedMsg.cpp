#include "PoseStampedMsg.hpp"

PoseStampedMsg::PoseStampedMsg()
{

}

PoseStampedMsg::~PoseStampedMsg()
{

}

msg_type PoseStampedMsg::getType()
{
    return msg_type::POSE;
}

const int PoseStampedMsg::getSize()
{
    return sizeof(PoseStampedMsg);
}