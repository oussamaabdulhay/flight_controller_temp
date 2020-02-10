#pragma once
#include "DataFilter.hpp"
#include <vector>

class ButterWorthFilter 
{
public:
    enum lpf_filter_coeff_fc_to_fs {lpf_ratio2_25,lpf_ratio1_50,lpf_ratio1_500,lpf_ratio1_1000,lpf_ratio1_2000};
    enum hpf_filter_coeff_fc_to_fs {hpf_ratio1_500,hpf_ratio1_2000};
    enum filter_type {lpf,hpf};
    void setCoefficients(lpf_filter_coeff_fc_to_fs);
    void setCoefficients(hpf_filter_coeff_fc_to_fs);
    void setFilterType(filter_type);
    void setFilterOrder(int);
    double filterData(double input);
    void setFilterInitialValue(double initial_value);
private:
    //Please refer to MATLAB butter implementation
    int filter_order;
    std::vector<double> prev_input={0};
    std::vector<double> prev_output={0};
    double low_pass_prev_input,low_pass_prev2_input;
    double low_pass_prev_output,low_pass_prev2_output;
    double high_pass_prev_input,high_pass_prev2_input;
    double high_pass_prev_output,high_pass_prev2_output;
    const std::vector<std::vector<double>> low_pass_2nd_b_coeff={{0.04613,0.09226,0.04613},{0.00362,0.00724,0.00362},{0.3913e-4,0.7826e-4,0.3913e-4},{0.0982e-4,0.1965e-4,0.0982e-4},{0.2462e-5,0.4924e-5,0.2462e-5}};
    const std::vector<std::vector<double>> low_pass_2nd_a_coeff={{1.,-1.30729,0.49181},{1,-1.8227,0.8372},{1,-1.9822,0.9824},{1,-1.9911,0.9911},{1,-1,9955,0.9956}};
    const std::vector<std::vector<double>> high_pass_2nd_b_coeff={{0.9912,-1.9823,0.9912},{0.9978,-1.9956,0.9978}};
    const std::vector<std::vector<double>> high_pass_2nd_a_coeff={{1.,-1.9822,0.9824},{1.,-1.9956,0.9956}};
    lpf_filter_coeff_fc_to_fs filter_coeff_lpf;
    hpf_filter_coeff_fc_to_fs filter_coeff_hpf;
    filter_type selected_filter_type;
};