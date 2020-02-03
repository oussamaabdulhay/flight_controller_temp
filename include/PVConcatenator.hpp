#pragma once
#include "MsgEmitter.hpp"
#include "MsgReceiver.hpp"
#include "Vector3DMessage.hpp"

class PVConcatenator : public msg_emitter, public msg_receiver{

private:
    Vector3D<double> pv_vector;

    control_system _cs;
    concatenation_axes selected_concatenation_axes;
public:
    enum concatenation_axes {conc_x_axis,conc_y_axis,conc_z_axis};
    enum receiving_channels {ch_broadcast,ch_pv,ch_pv_dot,ch_pv_dot_dot};
    void setConcatenationAxes(concatenation_axes); //TODO implement
    void receive_msg_data(DataMessage*);
    void receive_msg_data(DataMessage*, int);
    void concatenate();

    PVConcatenator(control_system);
    ~PVConcatenator();
};