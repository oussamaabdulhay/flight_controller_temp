#pragma once
#include "DataFilter.hpp"
#include "MsgEmitter.hpp"
#include "MsgReceiver.hpp"
#include "FloatMsg.hpp"

class ComplementaryFilterNoIntegration : public msg_receiver, public msg_emitter
{

public:
    enum low_pass_filter_coeff_fc_to_fs {ratio2_25};
    enum high_pass_filter_coeff_fc_to_fs {ratio1_500};
    enum receiver_channels {sensor_low_pass_reading,sensor_high_pass_reading};
    void set_settings(low_pass_filter_coeff_fc_to_fs,high_pass_filter_coeff_fc_to_fs);
	ComplementaryFilterNoIntegration();
    void receive_msg_data(DataMessage* t_msg,int);
private:
    //Please refer to MATLAB butter implementation
    double low_pass_prev_input,low_pass_prev2_input;
    double low_pass_prev_output,low_pass_prev2_output;
    double high_pass_prev_input,high_pass_prev2_input;
    double high_pass_prev_output,high_pass_prev2_output;
    const double low_pass_2nd_b_coeff[1][3]={{0.04613,0.09226,0.04613}};
    const double low_pass_2nd_a_coeff[1][3]={{1.,-1.30729,0.49181}};
    const double high_pass_2nd_b_coeff[1][3]={{0.9912,-1.9823,0.9912}};
    const double high_pass_2nd_a_coeff[1][3]={{1.,-1.9822,0.9824}};
    low_pass_filter_coeff_fc_to_fs lpf_coeff;
    high_pass_filter_coeff_fc_to_fs hpf_coeff;
};