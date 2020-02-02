#include "ButterWorthFilter.hpp"

void ButterWorthFilter::setCoefficients(lpf_filter_coeff_fc_to_fs t_filter_coeff){
    filter_coeff_lpf=t_filter_coeff;
}

void ButterWorthFilter::setCoefficients(hpf_filter_coeff_fc_to_fs t_filter_coeff){
    filter_coeff_hpf=t_filter_coeff;
}

void ButterWorthFilter::setFilterType(filter_type t_filter_type){
    selected_filter_type=t_filter_type;
}

double ButterWorthFilter::filterData(double input){
    double filter_output;
    if (filter_order==2){
        if (selected_filter_type==filter_type::lpf){
            filter_output=-low_pass_2nd_a_coeff[(int)filter_coeff_lpf][1]*prev_output[0]-low_pass_2nd_a_coeff[(int)filter_coeff_lpf][2]*prev_output[1]
            +input*low_pass_2nd_b_coeff[(int)filter_coeff_lpf][0]+low_pass_2nd_b_coeff[(int)filter_coeff_lpf][1]*prev_input[0]+
            low_pass_2nd_b_coeff[(int)filter_coeff_lpf][2]*prev_input[1];
        }
        else if (selected_filter_type==filter_type::hpf){
            filter_output=-high_pass_2nd_a_coeff[(int)filter_coeff_lpf][1]*prev_output[0]-high_pass_2nd_a_coeff[(int)filter_coeff_lpf][2]*prev_output[1]
            +input*high_pass_2nd_b_coeff[(int)filter_coeff_lpf][0]+high_pass_2nd_b_coeff[(int)filter_coeff_lpf][1]*prev_input[0]+
            high_pass_2nd_b_coeff[(int)filter_coeff_lpf][2]*prev_input[1];
        }

    }
    for (int i=filter_order-1;i>0;i--){
        prev_output[i]=prev_output[i-1];
        prev_input[i]=prev_input[i-1];
    }
    prev_output[0]=filter_output;
    prev_input[0]=input;
    return filter_output;
}

void ButterWorthFilter::setFilterOrder(int n_order){
    prev_input.clear();
    prev_output.clear();
    for (int i=0;i<n_order;i++){
        prev_input.push_back(0);
        prev_output.push_back(0);
    }
    filter_order=n_order;
}

void ButterWorthFilter::setFilterInitialValue(double initial_value){
    for (int i=0;i<filter_order;i++){
        prev_input[i]=initial_value;
        prev_output[i]=initial_value;
    }
}