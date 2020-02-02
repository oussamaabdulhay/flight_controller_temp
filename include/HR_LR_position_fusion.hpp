#pragma once
#include "thread_initial_unit.hpp"
#include "Vector3DMessage.hpp"
#include "ButterWorthFilter.hpp"
#include "Vector3D.hpp"
#include <vector>

class HR_LR_position_fusion : public thread_initial_unit
{
public:
    void runTasks() override;
    enum operation_mode {complementary_filter,bias_elimination};
    operation_mode current_operation_mode=bias_elimination; // TODO: complementary_filter is not implemented
    HR_LR_position_fusion();
private:
    const int num_filteration_ch=3;
    std::vector<ButterWorthFilter> three_axis_lpf;
    std::vector<ButterWorthFilter> three_axis_hpf; 
    bool is_first_reading=true;
    Vector3D<double> last_RTK_position_reading;
    Vector3D<double> last_XSens_position_reading;
    Vector3D<double> filtered_diff_position;
};