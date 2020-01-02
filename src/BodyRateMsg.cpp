#include "BodyRateMsg.hpp"

msg_type BodyRateMsg::getType()
{
	return msg_type::BODYRATE;
}

const int BodyRateMsg::getSize()
{
	return sizeof(BodyRateMsg);
}