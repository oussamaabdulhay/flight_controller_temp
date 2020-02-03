#pragma once
#include "MsgEmitter.hpp"
#include "MsgReceiver.hpp"
#include "Vector3DMessage.hpp"

class PVConcatenator : public msg_emitter, public msg_receiver{


public:
    enum concatenation_axes {conc_x_axis,conc_y_axis,conc_z_axis};
    enum receiving_channels {ch_broadcast,ch_pv,ch_pv_dot,ch_pv_dot_dot};
    void setConcatenationAxes(concatenation_axes); //TODO implement
    void receive_msg_data(DataMessage*);
    void receive_msg_data(DataMessage*, int);

    PVConcatenator(concatenation_axes);
    ~PVConcatenator();

private:
    Vector3D<double> pv_vector;
    concatenation_axes _selected_concatenation_axes;
};