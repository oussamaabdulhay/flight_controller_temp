#include "VectorDoubleMsg.hpp"

VectorDoubleMsg::VectorDoubleMsg()
{
}

VectorDoubleMsg::~VectorDoubleMsg()
{

}

msg_type VectorDoubleMsg::getType()
{
    return msg_type::VECTORDOUBLE;
}

const int VectorDoubleMsg::getSize()
{
    return sizeof(VectorDoubleMsg);
}