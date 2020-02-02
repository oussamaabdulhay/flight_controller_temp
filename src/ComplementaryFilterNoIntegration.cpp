#include "ComplementaryFilterNoIntegration.hpp"
void ComplementaryFilterNoIntegration::receive_msg_data(DataMessage* t_msg,int channel){
    if (t_msg->getType()==msg_type::DOUBLE){
        if (channel==(int)receiver_channels::sensor_low_pass_reading){
            double filter_output;
            filter_output=-low_pass_2nd_a_coeff[(int)lpf_coeff][1]*low_pass_prev_output-low_pass_2nd_a_coeff[(int)lpf_coeff][2]*low_pass_prev2_output
            +((FloatMsg*)t_msg)->data*low_pass_2nd_b_coeff[(int)lpf_coeff][0]+low_pass_2nd_b_coeff[(int)lpf_coeff][1]*low_pass_prev_input+low_pass_2nd_b_coeff[(int)lpf_coeff][2]*low_pass_prev2_input;
            low_pass_prev2_input=low_pass_prev_input;
            low_pass_prev_input=((FloatMsg*)t_msg)->data;
            low_pass_prev2_output=low_pass_prev_output;
            low_pass_prev_output=filter_output;
        }
        else if (channel==(int)receiver_channels::sensor_high_pass_reading){
            double filter_output;
            filter_output=-high_pass_2nd_a_coeff[(int)hpf_coeff][1]*high_pass_prev_output-high_pass_2nd_a_coeff[(int)hpf_coeff][2]*high_pass_prev2_output
            +((FloatMsg*)t_msg)->data*high_pass_2nd_b_coeff[(int)hpf_coeff][0]+high_pass_2nd_b_coeff[(int)hpf_coeff][1]*high_pass_prev_input+high_pass_2nd_b_coeff[(int)hpf_coeff][2]*high_pass_prev2_input;
            high_pass_prev2_input=high_pass_prev_input;
            high_pass_prev_input=((FloatMsg*)t_msg)->data;
            high_pass_prev2_output=high_pass_prev_output;
            high_pass_prev_output=filter_output;
        }
    }

}