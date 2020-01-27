#pragma once
#include "DataMessage.hpp"
#include "Vector3D.hpp"
#include "PoseMsg.hpp"
#include "TaggedPoses.hpp"
#include <vector>

class TaggedPosesMsg : public DataMessage
{

public:

    TaggedPosesMsg();
    ~TaggedPosesMsg();
    msg_type getType();
    const int getSize();
    DataMessage* Clone(){ return new TaggedPosesMsg(*this); }
    TaggedPoses tagged_poses;
};