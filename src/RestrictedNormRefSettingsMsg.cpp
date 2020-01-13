#include "RestrictedNormRefSettingsMsg.hpp"

    msg_type RestrictedNormRefSettingsMsg::getType(){
        return msg_type::RESTNORMREF_SETTINGS;
    }
    const int RestrictedNormRefSettingsMsg::getSize(){
        return sizeof(RestrictedNormRefSettingsMsg);
    }
    float RestrictedNormRefSettingsMsg::getMaxNorm(){
        return max_norm;
    }
    

    void RestrictedNormRefSettingsMsg::setMaxNorm(float t_max_norm){
        max_norm=t_max_norm;
    }
    DataMessage* RestrictedNormRefSettingsMsg::Clone(){ 
        return new RestrictedNormRefSettingsMsg(*this); 
    }
