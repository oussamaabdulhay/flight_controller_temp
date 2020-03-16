#include <iostream>
#include <vector>
#include "ROSUnit_Optitrack.hpp"
#include "std_logger.hpp"
#include "HexaActuationSystem.hpp"
#include "ESCMotor.hpp"
#include "ROSUnit_Arm.hpp"
#include "ROSUnit_UpdateController.hpp"
#include "ROSUnit_ResetController.hpp"
#include "ROSUnit_BroadcastData.hpp"
#include "ROSUnit_SwitchBlock.hpp"
#include "MRFTController.hpp"
#include "ROSUnit_Xsens.hpp"
#include "Transform_InertialToBody.hpp"
#include "RestrictedNormWaypointRefGenerator.hpp"
#include "ROSUnit_RestNormSettings.hpp"
#include "Saturation.hpp"
#include "CircularProcessVariableReference.hpp"
#include "Global2Inertial.hpp"
#include "ROSUnit_Factory.hpp"
#include "BatteryMonitor.hpp"
#include "ROSUnit_RTK.hpp"
#include "Differentiator.hpp"
#include "PVConcatenator.hpp"
#include "WrapAroundFunction.hpp"

#define XSENS_OVER_ROS
#define OPTITRACK
#undef BATTERY_MONITOR


const int PWM_FREQUENCY = 50;
const float SATURATION_VALUE_XY = 0.5;
const float SATURATION_VALUE_YAW = 1.0;
const float SATURATION_VALUE_YAWRATE = 0.3;


int main(int argc, char** argv) {
    //TODO remove SwitchOut Message
    std::cout << "Hello Flight Controller!" << std::endl;

    //****************************ROS UNITS*******************************

    ros::init(argc, argv, "flight_controller_node");

    ros::NodeHandle nh;
    ros::Rate rate(120);
    ROSUnit_Factory ROSUnit_Factory_main{nh};

    
    ROSUnit* myROSArm = new ROSUnit_Arm(nh);
    ROSUnit* myROSUpdateController = new ROSUnit_UpdateController(nh);
    ROSUnit* myROSResetController = new ROSUnit_ResetController(nh);
    ROSUnit* myROSBroadcastData = new ROSUnit_BroadcastData(nh);
    ROSUnit* myROSSwitchBlock = new ROSUnit_SwitchBlock(nh);
    ROSUnit* myROSRestNormSettings = new ROSUnit_RestNormSettings(nh);
    ROSUnit* ROSUnit_uav_control_set_path = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,ROSUnit_msg_type::ROSUnit_Poses,"uav_control/set_path");
    
    ROSUnit* rosunit_x_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/x");
    ROSUnit* rosunit_y_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/y");
    ROSUnit* rosunit_z_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/z");
    ROSUnit* rosunit_roll_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/roll");
    ROSUnit* rosunit_pitch_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/pitch");
    ROSUnit* rosunit_yaw_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/yaw");
    ROSUnit* rosunit_yaw_rate_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/yaw_rate");
                                                                

    //**************************SETTING BLOCKS**********************************

    Block* PID_x = new PIDController(block_id::PID_X);
    Block* PID_pitch = new PIDController(block_id::PID_PITCH);
    Block* PV_Ref_x = new ProcessVariableReference(block_id::REF_X);
    Block* PV_Ref_pitch = new ProcessVariableReference(block_id::REF_PITCH);
    Block* PID_y = new PIDController(block_id::PID_Y);
    Block* PID_roll = new PIDController(block_id::PID_ROLL);
    Block* PV_Ref_y = new ProcessVariableReference(block_id::REF_Y);
    Block* PV_Ref_roll = new ProcessVariableReference(block_id::REF_ROLL);
    Block* PID_z = new PIDController(block_id::PID_Z);
    Block* PID_yaw = new PIDController(block_id::PID_YAW);
    Block* PID_yaw_rate = new PIDController(block_id::PID_YAW_RATE);
    Block* PV_Ref_z = new ProcessVariableReference(block_id::REF_Z);
    Block* PV_Ref_yaw = new CircularProcessVariableReference(block_id::REF_YAW);
    Block* PV_Ref_yaw_rate = new ProcessVariableReference(block_id::REF_YAW_RATE);

    Block* MRFT_x = new MRFTController(block_id::MRFT_X);
    Block* MRFT_y = new MRFTController(block_id::MRFT_Y);
    Block* MRFT_z = new MRFTController(block_id::MRFT_Z);
    Block* MRFT_roll = new MRFTController(block_id::MRFT_ROLL);
    Block* MRFT_pitch = new MRFTController(block_id::MRFT_PITCH);
    Block* MRFT_yaw = new MRFTController(block_id::MRFT_YAW);
    Block* MRFT_yaw_rate = new MRFTController(block_id::MRFT_YAW_RATE);

    Transform_InertialToBody* transform_X_InertialToBody = new Transform_InertialToBody(control_system::x);
    Transform_InertialToBody* transform_Y_InertialToBody = new Transform_InertialToBody(control_system::y);

    RestrictedNormWaypointRefGenerator* myWaypoint = new RestrictedNormWaypointRefGenerator();

    Saturation* X_Saturation = new Saturation(SATURATION_VALUE_XY);
    Saturation* Y_Saturation = new Saturation(SATURATION_VALUE_XY);
    Saturation* Yaw_Saturation = new Saturation(SATURATION_VALUE_YAW);
    Saturation* YawRate_Saturation = new Saturation(SATURATION_VALUE_YAWRATE);


    //***********************SETTING CONTROL SYSTEMS***************************

    ControlSystem* X_ControlSystem = new ControlSystem(control_system::x, block_frequency::hz120);
    X_ControlSystem->addBlock(PID_x);
    X_ControlSystem->addBlock(MRFT_x);
    X_ControlSystem->addBlock(PV_Ref_x);

    ControlSystem* Pitch_ControlSystem = new ControlSystem(control_system::pitch, block_frequency::hz100);
    Pitch_ControlSystem->addBlock(PID_pitch);
    Pitch_ControlSystem->addBlock(MRFT_pitch);
    Pitch_ControlSystem->addBlock(PV_Ref_pitch);
    
    ControlSystem* Y_ControlSystem = new ControlSystem(control_system::y, block_frequency::hz120);
    Y_ControlSystem->addBlock(PID_y);
    Y_ControlSystem->addBlock(MRFT_y);
    Y_ControlSystem->addBlock(PV_Ref_y);

    ControlSystem* Roll_ControlSystem = new ControlSystem(control_system::roll, block_frequency::hz100);
    Roll_ControlSystem->addBlock(PID_roll);
    Roll_ControlSystem->addBlock(MRFT_roll);
    Roll_ControlSystem->addBlock(PV_Ref_roll);
    
    ControlSystem* Z_ControlSystem = new ControlSystem(control_system::z, block_frequency::hz120);
    Z_ControlSystem->addBlock(PID_z);
    Z_ControlSystem->addBlock(MRFT_z);
    Z_ControlSystem->addBlock(PV_Ref_z);

    ControlSystem* Yaw_ControlSystem = new ControlSystem(control_system::yaw, block_frequency::hz120);
    Yaw_ControlSystem->addBlock(PID_yaw);
    Yaw_ControlSystem->addBlock(MRFT_yaw);
    Yaw_ControlSystem->addBlock(PV_Ref_yaw);

    ControlSystem* YawRate_ControlSystem = new ControlSystem(control_system::yaw_rate, block_frequency::hz120);
    YawRate_ControlSystem->addBlock(PID_yaw_rate);
    YawRate_ControlSystem->addBlock(MRFT_yaw_rate);
    YawRate_ControlSystem->addBlock(PV_Ref_yaw_rate);

    //*********************SETTING ACTUATION SYSTEMS************************
    
    Actuator* M1 = new ESCMotor(0, PWM_FREQUENCY);
    Actuator* M2 = new ESCMotor(1, PWM_FREQUENCY);
    Actuator* M3 = new ESCMotor(2, PWM_FREQUENCY);
    Actuator* M4 = new ESCMotor(3, PWM_FREQUENCY);
    Actuator* M5 = new ESCMotor(4, PWM_FREQUENCY);
    Actuator* M6 = new ESCMotor(5, PWM_FREQUENCY);

    std::vector<Actuator*> actuators{M1, M2, M3, M4, M5, M6};

    ActuationSystem* myActuationSystem = new HexaActuationSystem(actuators);

    //***********************************SETTING CONNECTIONS***********************************
    //========                                                                                =============
    //|      |--x--->X_Control_System-->RM_X-->Saturation-->Roll_Control_System-------------->|           |
    //| USER |--x--->Y_Control_System-->RM_Y-->Saturation-->Pitch_Control_System------------->| Actuation |
    //|      |--x--->Z_Control_System-------------------------------------------------------->|  System   |
    //|      |--x--->Yaw_Control_System-->Saturation--->YawRate_Control_System-->Saturation-->|           |
    //========                                                                                =============
    
    Roll_ControlSystem->setEmittingChannel((int)HexaActuationSystem::receiving_channels::ch_roll);
    Pitch_ControlSystem->setEmittingChannel((int)HexaActuationSystem::receiving_channels::ch_pitch);
    Z_ControlSystem->setEmittingChannel((int)HexaActuationSystem::receiving_channels::ch_throttle);
    YawRate_ControlSystem->setEmittingChannel((int)HexaActuationSystem::receiving_channels::ch_yaw);

    X_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)transform_X_InertialToBody, (int)ControlSystem::unicast_addresses::unicast_control_system);
    transform_X_InertialToBody->addCallbackMsgReceiver((MsgReceiver*)X_Saturation);
    X_Saturation->addCallbackMsgReceiver((MsgReceiver*)Roll_ControlSystem);
    Roll_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)myActuationSystem, (int)ControlSystem::unicast_addresses::unicast_actuation_system);
    Y_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)transform_Y_InertialToBody, (int)ControlSystem::unicast_addresses::unicast_control_system);
    transform_Y_InertialToBody->addCallbackMsgReceiver((MsgReceiver*)Y_Saturation);
    Y_Saturation->addCallbackMsgReceiver((MsgReceiver*)Pitch_ControlSystem);
    Pitch_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)myActuationSystem, (int)ControlSystem::unicast_addresses::unicast_actuation_system);
    Z_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)myActuationSystem, (int)ControlSystem::unicast_addresses::unicast_actuation_system);
    Yaw_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)Yaw_Saturation, (int)ControlSystem::unicast_addresses::unicast_control_system);
    Yaw_Saturation->addCallbackMsgReceiver((MsgReceiver*)YawRate_ControlSystem);
    YawRate_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)YawRate_Saturation, (int)ControlSystem::unicast_addresses::unicast_actuation_system);
    YawRate_Saturation->addCallbackMsgReceiver((MsgReceiver*)myActuationSystem);

    //The saturation for YawRate was added because of the spikes that happen on the differentiation of Yaw when it jumps from
    //PI/2 to -PI/2. The value of 0.3 was based on the observation of the YawRate outputs on an actual flight.
    //Considering that the output on a normal flight never exceeded 0.25, the value 0.3 was chosen.

    
    //***********************SETTING FLIGHT SCENARIO INPUTS****************************
    myROSUpdateController->addCallbackMsgReceiver((MsgReceiver*)PID_x);
    myROSUpdateController->addCallbackMsgReceiver((MsgReceiver*)PID_y);
    myROSUpdateController->addCallbackMsgReceiver((MsgReceiver*)PID_z);
    myROSUpdateController->addCallbackMsgReceiver((MsgReceiver*)PID_roll);
    myROSUpdateController->addCallbackMsgReceiver((MsgReceiver*)PID_pitch);
    myROSUpdateController->addCallbackMsgReceiver((MsgReceiver*)PID_yaw);
    myROSUpdateController->addCallbackMsgReceiver((MsgReceiver*)PID_yaw_rate);

    myROSUpdateController->addCallbackMsgReceiver((MsgReceiver*)MRFT_x);
    myROSUpdateController->addCallbackMsgReceiver((MsgReceiver*)MRFT_y);
    myROSUpdateController->addCallbackMsgReceiver((MsgReceiver*)MRFT_z);
    myROSUpdateController->addCallbackMsgReceiver((MsgReceiver*)MRFT_roll);
    myROSUpdateController->addCallbackMsgReceiver((MsgReceiver*)MRFT_pitch);
    myROSUpdateController->addCallbackMsgReceiver((MsgReceiver*)MRFT_yaw);
    myROSUpdateController->addCallbackMsgReceiver((MsgReceiver*)MRFT_yaw_rate);

    myROSResetController->addCallbackMsgReceiver((MsgReceiver*)PID_x);
    myROSResetController->addCallbackMsgReceiver((MsgReceiver*)PID_y);
    myROSResetController->addCallbackMsgReceiver((MsgReceiver*)PID_z);
    myROSResetController->addCallbackMsgReceiver((MsgReceiver*)PID_roll);
    myROSResetController->addCallbackMsgReceiver((MsgReceiver*)PID_pitch);
    myROSResetController->addCallbackMsgReceiver((MsgReceiver*)PID_yaw);
    myROSResetController->addCallbackMsgReceiver((MsgReceiver*)PID_yaw_rate);

    myROSResetController->addCallbackMsgReceiver((MsgReceiver*)MRFT_x);
    myROSResetController->addCallbackMsgReceiver((MsgReceiver*)MRFT_y);
    myROSResetController->addCallbackMsgReceiver((MsgReceiver*)MRFT_z);
    myROSResetController->addCallbackMsgReceiver((MsgReceiver*)MRFT_roll);
    myROSResetController->addCallbackMsgReceiver((MsgReceiver*)MRFT_pitch);
    myROSResetController->addCallbackMsgReceiver((MsgReceiver*)MRFT_yaw);
    myROSResetController->addCallbackMsgReceiver((MsgReceiver*)MRFT_yaw_rate);

    myROSSwitchBlock->addCallbackMsgReceiver((MsgReceiver*)X_ControlSystem);
    myROSSwitchBlock->addCallbackMsgReceiver((MsgReceiver*)Y_ControlSystem);
    myROSSwitchBlock->addCallbackMsgReceiver((MsgReceiver*)Z_ControlSystem);
    myROSSwitchBlock->addCallbackMsgReceiver((MsgReceiver*)Roll_ControlSystem);
    myROSSwitchBlock->addCallbackMsgReceiver((MsgReceiver*)Pitch_ControlSystem);
    myROSSwitchBlock->addCallbackMsgReceiver((MsgReceiver*)Yaw_ControlSystem);
    myROSSwitchBlock->addCallbackMsgReceiver((MsgReceiver*)YawRate_ControlSystem);

    myROSArm->addCallbackMsgReceiver((MsgReceiver*) myActuationSystem);
    ROSUnit_uav_control_set_path->addCallbackMsgReceiver((MsgReceiver*)myWaypoint);
    myROSRestNormSettings->addCallbackMsgReceiver((MsgReceiver*)myWaypoint);

    
    //********************SETTING FLIGHT SCENARIO OUTPUTS***************************

    myActuationSystem->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData, (int)HexaActuationSystem::unicast_addresses::unicast_ActuationSystem_commands);
    myActuationSystem->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData, (int)HexaActuationSystem::unicast_addresses::unicast_ActuationSystem_armed);

    X_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);
    Y_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);
    Z_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);
    Roll_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);
    Pitch_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);
    Yaw_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);
    YawRate_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);
    myWaypoint->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);
    #ifdef BATTERY_MONITOR
    myBatteryMonitor->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);
    #endif
    MsgEmitter error_emitter;
    error_emitter.addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);
    //***********************INERTIAL TO BODY PROVIDER*****************************
 
    //TODO fix this
    //CsYaw_PVConcatenator->addCallbackMsgReceiver((MsgReceiver*)transform_X_InertialToBody);
    //CsYaw_PVConcatenator->addCallbackMsgReceiver((MsgReceiver*)transform_Y_InertialToBody);

    //***********************SETTING PID INITIAL VALUES*****************************

    ControllerMessage ctrl_msg;
    PID_parameters pid_para_init;

    pid_para_init.id = block_id::PID_X;
    ctrl_msg.setPIDParam(pid_para_init);
    ctrl_msg.set_dt(X_ControlSystem->get_dt());
    myROSUpdateController->emitMsgUnicastDefault((DataMessage*) &ctrl_msg);

    pid_para_init.id = block_id::PID_Y;
    ctrl_msg.setPIDParam(pid_para_init);
    ctrl_msg.set_dt(Y_ControlSystem->get_dt());
    myROSUpdateController->emitMsgUnicastDefault((DataMessage*) &ctrl_msg);

    pid_para_init.id = block_id::PID_Z;
    ctrl_msg.setPIDParam(pid_para_init);
    ctrl_msg.set_dt(Z_ControlSystem->get_dt());
    myROSUpdateController->emitMsgUnicastDefault((DataMessage*) &ctrl_msg);

    pid_para_init.id = block_id::PID_ROLL;
    ctrl_msg.setPIDParam(pid_para_init);
    ctrl_msg.set_dt(Roll_ControlSystem->get_dt());
    myROSUpdateController->emitMsgUnicastDefault((DataMessage*) &ctrl_msg);

    pid_para_init.id = block_id::PID_PITCH;
    ctrl_msg.setPIDParam(pid_para_init);
    ctrl_msg.set_dt(Pitch_ControlSystem->get_dt());
    myROSUpdateController->emitMsgUnicastDefault((DataMessage*) &ctrl_msg);

    pid_para_init.id = block_id::PID_YAW;
    ctrl_msg.setPIDParam(pid_para_init);
    ctrl_msg.set_dt(Yaw_ControlSystem->get_dt());
    myROSUpdateController->emitMsgUnicastDefault((DataMessage*) &ctrl_msg);

    pid_para_init.id = block_id::PID_YAW_RATE;
    ctrl_msg.setPIDParam(pid_para_init);
    ctrl_msg.set_dt(YawRate_ControlSystem->get_dt());
    myROSUpdateController->emitMsgUnicastDefault((DataMessage*) &ctrl_msg);

    //***********************SETTING MRFT INITIAL VALUES*****************************

    MRFT_parameters mrft_para_init;

    mrft_para_init.id = block_id::MRFT_X;
    ctrl_msg.setMRFTParam(mrft_para_init);
    ctrl_msg.set_dt(X_ControlSystem->get_dt());
    myROSUpdateController->emitMsgUnicastDefault((DataMessage*) &ctrl_msg);

    mrft_para_init.id = block_id::MRFT_Y;
    ctrl_msg.setMRFTParam(mrft_para_init);
    ctrl_msg.set_dt(Y_ControlSystem->get_dt());
    myROSUpdateController->emitMsgUnicastDefault((DataMessage*) &ctrl_msg);

    mrft_para_init.id = block_id::MRFT_Z;
    ctrl_msg.setMRFTParam(mrft_para_init);
    ctrl_msg.set_dt(Z_ControlSystem->get_dt());
    myROSUpdateController->emitMsgUnicastDefault((DataMessage*) &ctrl_msg);

    mrft_para_init.id = block_id::MRFT_ROLL;
    ctrl_msg.setMRFTParam(mrft_para_init);
    ctrl_msg.set_dt(Roll_ControlSystem->get_dt());
    myROSUpdateController->emitMsgUnicastDefault((DataMessage*) &ctrl_msg);

    mrft_para_init.id = block_id::MRFT_PITCH;
    ctrl_msg.setMRFTParam(mrft_para_init);
    ctrl_msg.set_dt(Pitch_ControlSystem->get_dt());
    myROSUpdateController->emitMsgUnicastDefault((DataMessage*) &ctrl_msg);

    mrft_para_init.id = block_id::MRFT_YAW;
    ctrl_msg.setMRFTParam(mrft_para_init);
    ctrl_msg.set_dt(Yaw_ControlSystem->get_dt());
    myROSUpdateController->emitMsgUnicastDefault((DataMessage*) &ctrl_msg);

    mrft_para_init.id = block_id::MRFT_YAW_RATE;
    ctrl_msg.setMRFTParam(mrft_para_init);
    ctrl_msg.set_dt(YawRate_ControlSystem->get_dt());
    myROSUpdateController->emitMsgUnicastDefault((DataMessage*) &ctrl_msg);

    
    while(ros::ok()){
        #ifdef BATTERY_MONITOR
        myBatteryMonitor->getVoltageReading();
        #endif
        ros::spinOnce();
        rate.sleep();
    }

    return 0;

}