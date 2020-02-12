#include "HR_LR_position_fusion.hpp"

void HR_LR_position_fusion::runTasks() {
    for (int i=0; i<thread_terminals.size();i++){
        if (thread_terminals[i]->getTerminalAddress()==thread_terminal_unit::RTK_pos){
            thread_terminals[i]->lock_mutex(); //TODO check if this can be moved to thread_terminal_unit
            DataMessage* RTK_msg=thread_terminals[i]->clone_last_message();
            thread_terminals[i]->unlock_mutex();
            last_RTK_position_reading=((Vector3DMessage*)RTK_msg)->getData();
            if (is_first_reading){
                is_first_reading=false;
                three_axis_lpf[0].setFilterInitialValue(last_RTK_position_reading.x);
                three_axis_lpf[1].setFilterInitialValue(last_RTK_position_reading.y);
                three_axis_lpf[2].setFilterInitialValue(last_RTK_position_reading.z);
                if (current_operation_mode==complementary_filter){
                    three_axis_hpf[0].setFilterInitialValue(last_RTK_position_reading.x);
                    three_axis_hpf[1].setFilterInitialValue(last_RTK_position_reading.y);
                    three_axis_hpf[2].setFilterInitialValue(last_RTK_position_reading.z);
                }
            }
            else {
                //std::cout<< "last_RTK_position_reading" << last_RTK_position_re1ading.x <<", "<<last_RTK_position_reading.y<<", " << last_RTK_position_reading.z << "\n";
                filtered_diff_position.x=three_axis_lpf[0].filterData(last_RTK_position_reading.x-last_XSens_position_reading.x);
                filtered_diff_position.y=three_axis_lpf[1].filterData(last_RTK_position_reading.y-last_XSens_position_reading.y);
                filtered_diff_position.z=three_axis_lpf[2].filterData(last_RTK_position_reading.z-last_XSens_position_reading.z);
                //std::cout<< "filtered_diff_position" << filtered_diff_position.x <<", "<<filtered_diff_position.y<<", " << filtered_diff_position.z << "\n";
            }
        }
        else if (thread_terminals[i]->getTerminalAddress()==thread_terminal_unit::XSens_pos){

            thread_terminals[i]->lock_mutex();
            DataMessage* XSens_msg=thread_terminals[i]->clone_last_message();
            thread_terminals[i]->unlock_mutex();
            last_XSens_position_reading=((Vector3DMessage*)XSens_msg)->getData();
        }
    }
    Vector3D<double> res;
    //res=last_XSens_position_reading+filtered_diff_position;
    //res=last_XSens_position_reading;
    res=last_RTK_position_reading;
    Vector3DMessage results;
    results.setVector3DMessage(res);
    emit_message_unicast(&results,(int)HR_LR_position_fusion::unicast_address::uni_pv_receiver,PVConcatenator::ch_pv);
    emit_message_unicast(&results,(int)HR_LR_position_fusion::unicast_address::uni_waypoint_receiver);
}

HR_LR_position_fusion::HR_LR_position_fusion(){
    for (int i=0; i<num_filteration_ch;i++){
        ButterWorthFilter fil;
        fil.setFilterOrder(2);
        fil.setFilterType(ButterWorthFilter::lpf);
        fil.setCoefficients(ButterWorthFilter::lpf_ratio1_500);
        three_axis_lpf.push_back(fil);
    }
    for (int i=0; i<num_filteration_ch;i++){
        ButterWorthFilter fil;
        fil.setFilterOrder(2);
        fil.setFilterType(ButterWorthFilter::hpf);
        fil.setCoefficients(ButterWorthFilter::hpf_ratio1_2000);
        three_axis_hpf.push_back(fil);
    }
}