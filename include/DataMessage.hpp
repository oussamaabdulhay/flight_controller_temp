
// Version: 2.0
// Author: M. Chehadeh
// Date: 29 Dec 2019
// Release note: added virtual clone constructor

#pragma once
#include <list>
#include "common_types.hpp"

class DataMessage {

public:
    
    virtual msg_type getType() = 0;
    virtual const int getSize() = 0;
    virtual DataMessage* Clone () = 0;
    DataMessage(){}
    ~DataMessage(){}
};