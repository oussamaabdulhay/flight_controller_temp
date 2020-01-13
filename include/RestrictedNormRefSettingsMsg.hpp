#pragma once
#include "DataMessage.hpp"
class RestrictedNormRefSettingsMsg : public DataMessage {

private:
    float max_norm;
    
public:
    bool enabled=true;//Disabling stops updating reference value
    bool delete_existing_waypoints=false;//Setting to true clears all uploaded waypoints. No retention.
    msg_type getType();
    const int getSize();
    float getMaxNorm();
    

    void setMaxNorm(float t_data);
    DataMessage* Clone();
};
