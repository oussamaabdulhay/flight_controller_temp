// Version: 1.1
// Author: Mohamad Chehadeh
// Date: 26 Jan 2020
// Revision Note: Moved from Outdoor Navigation to UAV Control
#pragma once

#include "MsgEmitter.hpp"
#include "MsgReceiver.hpp"
#include "OptitrackMessage.hpp"
#include "Vector3D.hpp"
#include "HeadingMsg.hpp"
#include "AttitudeMsg.hpp"
#include "RotationMatrix3by3.hpp"
#include "RTKMessagePosition.hpp"
#include "RTKMessageVelocity.hpp"
#include "PoseStampedMsg.hpp"
#include <math.h>
#include "FloatMsg.hpp"
#include "Vector3DMessage.hpp"

class Global2Inertial : public msg_emitter, public msg_receiver
{
private:
    Vector3D<double> calib_point1, calib_point2;
    double calibrated_global_to_inertial_angle;
    double calibrated_reference_inertial_heading;//TODO: This needs to be moved to XSens node
    Vector3D<double> antenna_pose;
    Vector3D<double> last_known_orientation;
    Vector3D<double> transformPoint(Vector3D<double>);
    Vector3D<double> getEulerfromQuaternion(Quaternion);
    HeadingMsg getHeading(Quaternion);
public:
enum unicast_addresses {broadcast,uni_RTK_pos,uni_XSens_pos,uni_Optitrack_pos,uni_XSens_vel};
enum receiving_channels {ch_broadcast,ch_RTK_pos,ch_XSens_pos,ch_Optitrack_pos,ch_XSens_vel};
    Global2Inertial();
    void receive_msg_data(DataMessage* t_msg);
    void receive_msg_data(DataMessage* t_msg,int ch);

};
