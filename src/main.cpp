#include <iostream>
#include <vector>
#include "ROSUnit_Optitrack.hpp"
#include "PIDController.hpp"
#include "ControlSystem.hpp"
#include "PID_values.hpp"
#include "ProcessVariableReference.hpp"
#include "ActuationSystem.hpp"
#include "std_logger.hpp"
#include "HexaActuationSystem.hpp"
#include "ESCMotor.hpp"
#include "ROSUnit_Arm.hpp"
#include "ROSUnit_UpdateController.hpp"
#include "ROSUnit_ResetController.hpp"
#include "ROSUnit_BroadcastData.hpp"
#include "ROSUnit_SwitchBlock.hpp"
#include "MRFTController.hpp"
#include "MRFT_values.hpp"
#include "ControllerMessage.hpp"
#include "ROSUnit_Xsens.hpp"
#include "XSens_IMU.hpp"
#include "Transform_InertialToBody.hpp"
#include "TimedBlock.hpp"
#include <iostream>
#include <stdexcept>
#include <string>
#include "CallbackHandler.hpp"
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
#include <thread>
#include "xsstatusflag.h"
#include "WrapAroundFunction.hpp"
#include "IntegerMsg.hpp"
#include "ROSUnit_Xsens.hpp"

#define DEBUG_HR_LR_DECOUPLED
#define XSENS_OVER_ROS
#undef RTK_GPS_FILTER
#undef XSENS_OVER_SERIAL
#undef RTK_ONLY
#undef XSENS_GPS_ONLY

#undef Navio_IMU_en
#define OPTITRACK
#undef BATTERY_MONITOR

const int OPTITRACK_FREQUENCY = 120;
const int PWM_FREQUENCY = 50;
const float SATURATION_VALUE_XY = 0.5;
const float SATURATION_VALUE_YAWRATE = 1.0;

ROSUnit* myROSBroadcastData;
MsgEmitter error_emitter;

Journaller *gJournal = 0;

#ifdef RTK_GPS_FILTER
void worker(TimedBlock* timed_block) {
    timed_block->tickTimer();
    usleep(timed_block->getLoopRemainingMicroSec());
    while(true){
        while(!timed_block->hasLoopTimeElapsed()){
        }
        timed_block->tickTimer();
        if(timed_block->getLoopRemainingMicroSec() < 0){
            Logger::getAssignedLogger()->log("exceeded loop time 100hz ",LoggerLevel::Warning);
            
            IntegerMsg error_msg; //TODO make enum of different errors
            error_msg.data = 1;
            error_emitter.emitMsgUnicastDefault((DataMessage*)&error_msg);
            
        } else {
            usleep(timed_block->getLoopRemainingMicroSec());
        }
    }
}
#endif

int main(int argc, char** argv) {
    
    std::cout << "Hello Easy C++ project!" << std::endl;
    //TODO separate files on specific folders
    //TODO make a single pattern to follow for providers
    //TODO makedirect connection between Xsens and Raspberry

    //*****************************ROS UNITS*******************************

    ros::init(argc, argv, "flight_controller_node");

    ros::NodeHandle nh;
    ros::Rate rate(100);
    ROSUnit_Factory ROSUnit_Factory_main{nh};

    ROSUnit* myROSOptitrack = new ROSUnit_Optitrack(nh);
    ROSUnit* myROSArm = new ROSUnit_Arm(nh);
    ROSUnit* myROSUpdateController = new ROSUnit_UpdateController(nh);
    ROSUnit* myROSResetController = new ROSUnit_ResetController(nh);
    ROSUnit* myROSBroadcastData = new ROSUnit_BroadcastData(nh);
    ROSUnit* myROSSwitchBlock = new ROSUnit_SwitchBlock(nh);
    ROSUnit* myROSRestNormSettings = new ROSUnit_RestNormSettings(nh);
    ROSUnit* ROSUnit_uav_control_set_path = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,ROSUnit_msg_type::ROSUnit_Poses,"uav_control/set_path");
    ROSUnit* ROSUnit_set_height_offset = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,ROSUnit_msg_type::ROSUnit_Float,"set_height_offset");
    ROSUnit* myROSRTK = new ROSUnit_RTK(nh);

    //*****************************LOGGER**********************************
    Logger::assignLogger(new StdLogger());
    //***********************ADDING SENSORS********************************
    #ifdef BATTERY_MONITOR
    BatteryMonitor* myBatteryMonitor = new BatteryMonitor();
    #endif
    #ifdef Navio_IMU_en
    NAVIOMPU9250_sensor* myIMU = new NAVIOMPU9250_sensor();
    myIMU->setSettings(ACCELEROMETER, FSR, 16);
    myIMU->setSettings(GYROSCOPE, FSR, 2000);
    myIMU->setSettings(MAGNETOMETER, FSR, 16);
    #endif
    
    
    #ifdef XSENS_OVER_SERIAL
    
    //For the XSens to wokr over USB/FTDI connection, it's necessary to install a program called "setserial" on Linux
    //Once installed, run the command "setserial /name/of/the/dev/port low_latency". Doing so, will reduce the buffering
    //from the the FTDI driver, and the data will arrive at almost constant rates.
    //https://base.xsens.com/hc/en-us/community/posts/360029341694-How-do-I-get-IMU-data-via-USB-in-real-time-

    cout << "Creating XsControl object..." << endl;
	XsControl* control = XsControl::construct();
	assert(control != 0);

	// Lambda function for error handling
	auto handleError = [=](string errorString)
	{
		control->destruct();
		cout << errorString << endl;
		cout << "Press [ENTER] to continue." << endl;
		cin.get();
		return -1;
	};

	cout << "Scanning for devices..." << endl;
	//XsPortInfoArray portInfoArray = XsScanner::scanPorts(XsBaudRate::XBR_460k8,100,false);
	// Find an MTi device
	XsPortInfo mtPort;
    
    XsString portName("/dev/ttyAMA0");
    cout << "Input: " << portName.toStdString() << " @ baud: " <<(int) XsBaudRate::XBR_460k8 << endl;

    XsPortInfo ManualmtPort=XsScanner::scanPort(portName, XsBaudRate::XBR_460k8);
    
    cout << "List: " << ManualmtPort.deviceId().toString().toStdString() << " @ port: " << ManualmtPort.portName().toStdString() << ", baudrate: " << ManualmtPort.baudrate() << endl;
    
    cout << "isMti: " << ManualmtPort.deviceId().isMti() << " @ MtiG: " << ManualmtPort.deviceId().isMtig() << endl;

    XsPortInfoArray portInfoArray; //TODO: Remove
    for (auto const &portInfo : portInfoArray)
	{
		if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
		{
			mtPort = portInfo;
			break;
		}
	    cout << "List: " << portInfo.deviceId().toString().toStdString() << " @ port: " << portInfo.portName().toStdString() << ", baudrate: " << portInfo.baudrate() << endl;
	}

	if (ManualmtPort.empty())
		return handleError("No MTi device found. Aborting.");

	cout << "Found a device with ID: " << ManualmtPort.deviceId().toString().toStdString() << " @ port: " << ManualmtPort.portName().toStdString() << ", baudrate: " << ManualmtPort.baudrate() << endl;

	cout << "Opening port..." << endl;
    if (!control->openPort(ManualmtPort.portName().toStdString(), ManualmtPort.baudrate())){//Timeout in ms
        return handleError("Could not open port. Aborting.");
    }


	// Get the device object

	XsDevice* device = control->device(ManualmtPort.deviceId());
	assert(device != 0);
	cout << "Device: " << device->productCode().toStdString() << ", with ID: " << device->deviceId().toString() << " opened." << endl;

	// Create and attach callbackXSens handler to device
	CallbackHandler callbackXSens;
	device->addCallbackHandler(&callbackXSens);

	cout << "Putting device into measurement mode..." << endl;
	if (!device->gotoMeasurement())
		return handleError("Could not put device into measurement mode. Aborting.");

    char ans;
    uint16_t ans_time;
    std::cout << "Do gyro calibration? (y/n)" << std::endl;
    std::cin >> ans;
    if(ans == 'y'){
        //************* GYRO CALIBRATION PROCEDURE *****************
        std::cout << "For how long? (recommended: 6) (s)" << std::endl;
        std::cin >> ans_time;
        XsTime::msleep(100); //Give the device a little bit of time to enter measurement mode
        uint16_t biasEstimateDuration = ans_time;
        device->setNoRotation(biasEstimateDuration);
        bool noRotationStarted = false;
        int64_t biasEstimateStart = XsTime::timeStampNow();
        while (true){
            if (callbackXSens.packetAvailable()){
                    XsDataPacket packet = callbackXSens.getNextPacket();
                    uint32_t noRotationStatus = packet.status() & XSF_NoRotationMask;

                if (!noRotationStarted){
                    if (noRotationStatus == XSF_NoRotationRunningNormally){
                        std::cout << "No rotation started"<< std::endl;
                        noRotationStarted = true;
                    }
                }
                else{
                    if (noRotationStatus == 0){
                        std::cout << "No rotation ended successfully"<< std::endl;
                        break;
                    }
                }


                if (noRotationStatus == XSF_NoRotationAborted){
                    std::cout << "No rotation aborted" << std::endl;
                    break;
                }

                if (noRotationStatus == XSF_NoRotationSamplesRejected){
                    std::cout << "No rotation ended with rejected samples" << std::endl;
                    break;
                }
            }

            //Check for timeout
            int64_t timeout = ((int64_t)biasEstimateDuration * 1000 + 500);
            if ( (XsTime::timeStampNow() - biasEstimateStart) > timeout)
            {
            if (!noRotationStarted)
            std::cout << "No rotation did not start in time" << std::endl;
            else
            std::cout << "No rotation did not end in time" << std::endl;
            break;
            }
            XsTime::msleep(0);
        }
        //************* END OF GYRO CALIBRATION PROCEDURE *****************
    }
    
    XSens_IMU* myXSensIMU = new XSens_IMU();

    #endif

    

    //***********************FILTER RTK_GPS*************************************
    #ifdef RTK_GPS_FILTER
    HR_LR_position_fusion* hr_lr_position_fusion = new HR_LR_position_fusion();
    hr_lr_position_fusion->setLoopFrequency(block_frequency::hz100);
    hr_lr_position_fusion->current_operation_mode = HR_LR_position_fusion::operation_mode::bias_elimination;

    thread_terminal_unit rtk_position_terminal_unit;
    thread_terminal_unit xsens_position_terminal_unit;

    rtk_position_terminal_unit.setTerminalUnitAddress(thread_terminal_unit::RTK_pos);
    xsens_position_terminal_unit.setTerminalUnitAddress(thread_terminal_unit::XSens_pos);
    ((thread_initial_unit*)hr_lr_position_fusion)->addTerminalUnit(&rtk_position_terminal_unit);
    ((thread_initial_unit*)hr_lr_position_fusion)->addTerminalUnit(&xsens_position_terminal_unit);
    #endif
    //***********************SETTING PROVIDERS**********************************
    
    Global2Inertial* myGlobal2Inertial = new Global2Inertial();
    PVConcatenator* CsX_PVConcatenator = new PVConcatenator(PVConcatenator::concatenation_axes::conc_x_axis, act_on::pv);
    PVConcatenator* CsY_PVConcatenator = new PVConcatenator(PVConcatenator::concatenation_axes::conc_y_axis, act_on::pv);
    PVConcatenator* CsZ_PVConcatenator = new PVConcatenator(PVConcatenator::concatenation_axes::conc_z_axis, act_on::pv);
    PVConcatenator* CsRoll_PVConcatenator = new PVConcatenator(PVConcatenator::concatenation_axes::conc_x_axis, act_on::pv);
    PVConcatenator* CsPitch_PVConcatenator = new PVConcatenator(PVConcatenator::concatenation_axes::conc_y_axis, act_on::pv);
    PVConcatenator* CsYaw_PVConcatenator = new PVConcatenator(PVConcatenator::concatenation_axes::conc_z_axis, act_on::pv);
    PVConcatenator* CsYawRate_PVConcatenator = new PVConcatenator(PVConcatenator::concatenation_axes::conc_z_axis, act_on::pv);

    WrapAroundFunction* wrap_around_yaw = new WrapAroundFunction();
    wrap_around_yaw->assignParametersRange(-M_PI, M_PI);
    
    #ifdef OPTITRACK
    Differentiator* velocityFromPosition = new Differentiator(1./OPTITRACK_FREQUENCY);
    velocityFromPosition->setEmittingChannel((int)PVConcatenator::receiving_channels::ch_pv_dot);
    
    Differentiator* yawRateFromYaw = new Differentiator(1./OPTITRACK_FREQUENCY);
    yawRateFromYaw->setEmittingChannel((int)PVConcatenator::receiving_channels::ch_pv);
    
    myROSOptitrack->addCallbackMsgReceiver((MsgReceiver*)myGlobal2Inertial);
    myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)velocityFromPosition, (int)Global2Inertial::unicast_addresses::uni_Optitrack_pos);
    myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)yawRateFromYaw, (int)Global2Inertial::unicast_addresses::uni_Optitrack_heading);
    velocityFromPosition->addCallbackMsgReceiver((MsgReceiver*)CsX_PVConcatenator);
    velocityFromPosition->addCallbackMsgReceiver((MsgReceiver*)CsY_PVConcatenator);
    velocityFromPosition->addCallbackMsgReceiver((MsgReceiver*)CsZ_PVConcatenator);
    yawRateFromYaw->addCallbackMsgReceiver((MsgReceiver*)CsYawRate_PVConcatenator);
    myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)CsX_PVConcatenator, (int)Global2Inertial::unicast_addresses::uni_Optitrack_pos);
    myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)CsY_PVConcatenator, (int)Global2Inertial::unicast_addresses::uni_Optitrack_pos);
    myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)CsZ_PVConcatenator, (int)Global2Inertial::unicast_addresses::uni_Optitrack_pos);
    myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)CsYaw_PVConcatenator, (int)Global2Inertial::unicast_addresses::uni_Optitrack_heading);
    #endif

    #ifdef XSENS_OVER_SERIAL
    callbackXSens.addCallbackMsgReceiver((MsgReceiver*)CsRoll_PVConcatenator,(int)CallbackHandler::unicast_addresses::unicast_XSens_attitude_rate);
    callbackXSens.addCallbackMsgReceiver((MsgReceiver*)CsPitch_PVConcatenator,(int)CallbackHandler::unicast_addresses::unicast_XSens_attitude_rate);
    callbackXSens.addCallbackMsgReceiver((MsgReceiver*)CsYawRate_PVConcatenator,(int)CallbackHandler::unicast_addresses::unicast_XSens_yaw_rate);
    callbackXSens.addCallbackMsgReceiver((MsgReceiver*)myGlobal2Inertial,(int)CallbackHandler::unicast_addresses::unicast_XSens_orientation);
    callbackXSens.addCallbackMsgReceiver((MsgReceiver*)myGlobal2Inertial,(int)CallbackHandler::unicast_addresses::unicast_XSens_translation);
    callbackXSens.addCallbackMsgReceiver((MsgReceiver*)myGlobal2Inertial,(int)CallbackHandler::unicast_addresses::unicast_XSens_translation_rate);
    #endif
    #ifdef XSENS_OVER_ROS
    ROSUnit* myROSUnit_Xsens = new ROSUnit_Xsens(nh);
    myROSUnit_Xsens->addCallbackMsgReceiver((MsgReceiver*)CsRoll_PVConcatenator,(int)ROSUnit_Xsens::unicast_addresses::unicast_XSens_attitude_rate);
    myROSUnit_Xsens->addCallbackMsgReceiver((MsgReceiver*)CsPitch_PVConcatenator,(int)ROSUnit_Xsens::unicast_addresses::unicast_XSens_attitude_rate);
    myROSUnit_Xsens->addCallbackMsgReceiver((MsgReceiver*)CsYawRate_PVConcatenator,(int)ROSUnit_Xsens::unicast_addresses::unicast_XSens_yaw_rate);
    myROSUnit_Xsens->addCallbackMsgReceiver((MsgReceiver*)myGlobal2Inertial,(int)ROSUnit_Xsens::unicast_addresses::unicast_XSens_orientation);
    myROSUnit_Xsens->addCallbackMsgReceiver((MsgReceiver*)myGlobal2Inertial,(int)ROSUnit_Xsens::unicast_addresses::unicast_XSens_translation);
    myROSUnit_Xsens->addCallbackMsgReceiver((MsgReceiver*)myGlobal2Inertial,(int)ROSUnit_Xsens::unicast_addresses::unicast_XSens_translation_rate);
    #endif

    myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)CsX_PVConcatenator, (int)Global2Inertial::unicast_addresses::uni_XSens_vel);
    myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)CsY_PVConcatenator, (int)Global2Inertial::unicast_addresses::uni_XSens_vel);
    myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)CsZ_PVConcatenator, (int)Global2Inertial::unicast_addresses::uni_XSens_vel);
    myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)wrap_around_yaw, (int)Global2Inertial::unicast_addresses::uni_XSens_ori);
    wrap_around_yaw->addCallbackMsgReceiver((MsgReceiver*)CsYaw_PVConcatenator);
    myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)CsRoll_PVConcatenator,(int)Global2Inertial::unicast_addresses::uni_XSens_ori); //TODO bad grouping with ifdef
    myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)CsPitch_PVConcatenator,(int)Global2Inertial::unicast_addresses::uni_XSens_ori);

    #ifdef RTK_GPS_FILTER
    myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)&rtk_position_terminal_unit, (int)Global2Inertial::unicast_addresses::uni_RTK_pos_pv);
    myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)&xsens_position_terminal_unit, (int)Global2Inertial::unicast_addresses::uni_XSens_pos);
    hr_lr_position_fusion->addCallbackMsgReceiver((MsgReceiver*)CsX_PVConcatenator,HR_LR_position_fusion::uni_pv_receiver);
    hr_lr_position_fusion->addCallbackMsgReceiver((MsgReceiver*)CsY_PVConcatenator,HR_LR_position_fusion::uni_pv_receiver);
    hr_lr_position_fusion->addCallbackMsgReceiver((MsgReceiver*)CsZ_PVConcatenator,HR_LR_position_fusion::uni_pv_receiver);
    thread* hr_lr_position_fusion_thread = new thread(worker, (TimedBlock*)hr_lr_position_fusion);
    #endif
    #ifdef RTK_ONLY
    myROSRTK->addCallbackMsgReceiver((MsgReceiver*)myGlobal2Inertial);
    myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)CsX_PVConcatenator, (int)Global2Inertial::unicast_addresses::uni_RTK_pos_pv);
    myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)CsY_PVConcatenator, (int)Global2Inertial::unicast_addresses::uni_RTK_pos_pv);
    myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)CsZ_PVConcatenator, (int)Global2Inertial::unicast_addresses::uni_RTK_pos_pv);
    #endif
    #ifdef XSENS_GPS_ONLY
    myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)CsX_PVConcatenator, (int)Global2Inertial::unicast_addresses::uni_XSens_pos);
    myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)CsY_PVConcatenator,(int) Global2Inertial::unicast_addresses::uni_XSens_pos);
    myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)CsZ_PVConcatenator, (int)Global2Inertial::unicast_addresses::uni_XSens_pos);
    #endif

    
    #ifdef Navio_IMU_en
    AccGyroAttitudeObserver myAttObserver((BodyAccProvider*) myIMU->getAcc(), 
                                          (BodyRateProvider*) myIMU->getGyro(),
                                          block_frequency::hhz200);

    
    
     ComplementaryFilter filter1, filter2, filter3;
    //TODO second argument should be dt of IMU sampling rate
     ComplementaryFilterSettings settings(false, 0.005, 0.995);

     myAttObserver.setFilterType(&filter1, &filter2);
     myAttObserver.updateSettings(&settings, 0.1);

     Roll_PVProvider* myRollPV = (Roll_PVProvider*) &myAttObserver;
     Pitch_PVProvider* myPitchPV = (Pitch_PVProvider*) &myAttObserver;
    #endif        

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

    //TODO make all of these Blocks
    // Vector3D<float>* inertial_command = new Vector3D<float>(); //TODO-Chehadeh
    Transform_InertialToBody* transform_X_InertialToBody = new Transform_InertialToBody(control_system::x);
    Transform_InertialToBody* transform_Y_InertialToBody = new Transform_InertialToBody(control_system::y);

    RestrictedNormWaypointRefGenerator* myWaypoint = new RestrictedNormWaypointRefGenerator();

    Saturation* X_Saturation = new Saturation(SATURATION_VALUE_XY);
    Saturation* Y_Saturation = new Saturation(SATURATION_VALUE_XY);
    Saturation* YawRate_Saturation = new Saturation(SATURATION_VALUE_YAWRATE);

    //***********************SETTING CONTROL SYSTEMS***************************
    //TODO Expose switcher to the main, add blocks to the switcher, then make connections between switcher, then add them to the Control System
    ControlSystem* X_ControlSystem = new ControlSystem(control_system::x, block_frequency::hz100);
    X_ControlSystem->addBlock(PID_x);
    X_ControlSystem->addBlock(MRFT_x);
    X_ControlSystem->addBlock(PV_Ref_x);

    ControlSystem* Pitch_ControlSystem = new ControlSystem(control_system::pitch, block_frequency::hz100);
    Pitch_ControlSystem->addBlock(PID_pitch);
    Pitch_ControlSystem->addBlock(MRFT_pitch);
    Pitch_ControlSystem->addBlock(PV_Ref_pitch);
    
    ControlSystem* Y_ControlSystem = new ControlSystem(control_system::y, block_frequency::hz100);
    Y_ControlSystem->addBlock(PID_y);
    Y_ControlSystem->addBlock(MRFT_y);
    Y_ControlSystem->addBlock(PV_Ref_y);

    ControlSystem* Roll_ControlSystem = new ControlSystem(control_system::roll, block_frequency::hz100);
    Roll_ControlSystem->addBlock(PID_roll);
    Roll_ControlSystem->addBlock(MRFT_roll);
    Roll_ControlSystem->addBlock(PV_Ref_roll);
    
    ControlSystem* Z_ControlSystem = new ControlSystem(control_system::z, block_frequency::hz100);
    Z_ControlSystem->addBlock(PID_z);
    Z_ControlSystem->addBlock(MRFT_z);
    Z_ControlSystem->addBlock(PV_Ref_z);

    ControlSystem* Yaw_ControlSystem = new ControlSystem(control_system::yaw, block_frequency::hz100);
    Yaw_ControlSystem->addBlock(PID_yaw);
    Yaw_ControlSystem->addBlock(MRFT_yaw);
    Yaw_ControlSystem->addBlock(PV_Ref_yaw);

    ControlSystem* YawRate_ControlSystem = new ControlSystem(control_system::yaw_rate, block_frequency::hz100);
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
    //========                                                                     =============
    //|      |--x--->X_Control_System-->RM_X-->Saturation-->Roll_Control_System--->|           |
    //| USER |--x--->Y_Control_System-->RM_Y-->Saturation-->Pitch_Control_System-->| Actuation |
    //|      |--x--->Z_Control_System--------------------------------------------->|  System   |
    //|      |--x--->Yaw_Control_System-->Saturation--->YawRate_Control_System---->|           |
    //========                                                                     =============
    
    X_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)transform_X_InertialToBody, (int)ControlSystem::unicast_addresses::unicast_control_system);
    transform_X_InertialToBody->addCallbackMsgReceiver((MsgReceiver*)X_Saturation);
    X_Saturation->addCallbackMsgReceiver((MsgReceiver*)Roll_ControlSystem);
    Roll_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)myActuationSystem);
    Y_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)transform_Y_InertialToBody, (int)ControlSystem::unicast_addresses::unicast_control_system);
    transform_Y_InertialToBody->addCallbackMsgReceiver((MsgReceiver*)Y_Saturation);
    Y_Saturation->addCallbackMsgReceiver((MsgReceiver*)Pitch_ControlSystem);
    Pitch_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)myActuationSystem);
    Z_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)myActuationSystem, (int)ControlSystem::unicast_addresses::unicast_control_system);
    Yaw_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)YawRate_Saturation, (int)ControlSystem::unicast_addresses::unicast_control_system);
    YawRate_Saturation->addCallbackMsgReceiver((MsgReceiver*)YawRate_ControlSystem);
    YawRate_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)myActuationSystem, (int)ControlSystem::unicast_addresses::unicast_control_system);

    //******************PROVIDERS TO CONTROL SYSTEMS******************************

    CsX_PVConcatenator->addCallbackMsgReceiver((MsgReceiver*)X_ControlSystem);
    CsY_PVConcatenator->addCallbackMsgReceiver((MsgReceiver*)Y_ControlSystem);
    CsZ_PVConcatenator->addCallbackMsgReceiver((MsgReceiver*)Z_ControlSystem);
    CsPitch_PVConcatenator->addCallbackMsgReceiver((MsgReceiver*)Pitch_ControlSystem);
    CsRoll_PVConcatenator->addCallbackMsgReceiver((MsgReceiver*)Roll_ControlSystem);
    CsYaw_PVConcatenator->addCallbackMsgReceiver((MsgReceiver*)Yaw_ControlSystem);
    CsYawRate_PVConcatenator->addCallbackMsgReceiver((MsgReceiver*)YawRate_ControlSystem);

    //******************SETTING TRAJECTORY GENERATION TOOL******************

    myWaypoint->add_x_control_system(X_ControlSystem);
    myWaypoint->add_y_control_system(Y_ControlSystem);
    myWaypoint->add_z_control_system(Z_ControlSystem);
    myWaypoint->add_yaw_control_system(Yaw_ControlSystem);
    
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
    #ifdef OPTITRACK
    myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)myWaypoint, (int)Global2Inertial::unicast_addresses::uni_Optitrack_pos); 
    #else
    myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)myWaypoint, (int)Global2Inertial::unicast_addresses::uni_RTK_pos_wp); 
    #endif
    ROSUnit_uav_control_set_path->addCallbackMsgReceiver((MsgReceiver*)myWaypoint);
    myROSRestNormSettings->addCallbackMsgReceiver((MsgReceiver*)myWaypoint);

    ROSUnit_set_height_offset->addCallbackMsgReceiver((MsgReceiver*)myGlobal2Inertial);
    
    //********************SETTING FLIGHT SCENARIO OUTPUTS***************************
    CsX_PVConcatenator->setEmittingChannel((int)ROSUnit_BroadcastData::ros_broadcast_channels::x);
    CsY_PVConcatenator->setEmittingChannel((int)ROSUnit_BroadcastData::ros_broadcast_channels::y);
    CsZ_PVConcatenator->setEmittingChannel((int)ROSUnit_BroadcastData::ros_broadcast_channels::z);
    CsRoll_PVConcatenator->setEmittingChannel((int)ROSUnit_BroadcastData::ros_broadcast_channels::roll);  
    CsPitch_PVConcatenator->setEmittingChannel((int)ROSUnit_BroadcastData::ros_broadcast_channels::pitch);
    CsYaw_PVConcatenator->setEmittingChannel((int)ROSUnit_BroadcastData::ros_broadcast_channels::yaw);
    CsYawRate_PVConcatenator->setEmittingChannel((int)ROSUnit_BroadcastData::ros_broadcast_channels::yaw_rate);

    CsX_PVConcatenator->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);
    CsY_PVConcatenator->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);
    CsZ_PVConcatenator->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);
    CsRoll_PVConcatenator->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);
    CsPitch_PVConcatenator->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);
    CsYaw_PVConcatenator->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);
    CsYawRate_PVConcatenator->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);

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
    error_emitter.addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);
    //***********************INERTIAL TO BODY PROVIDER*****************************
 
    CsYaw_PVConcatenator->addCallbackMsgReceiver((MsgReceiver*)transform_X_InertialToBody);
    CsYaw_PVConcatenator->addCallbackMsgReceiver((MsgReceiver*)transform_Y_InertialToBody);

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

    #ifdef XSENS_OVER_SERIAL
	cout << "Closing port..." << endl;
	control->closePort(ManualmtPort.portName().toStdString());

	cout << "Freeing XsControl object..." << endl;
	control->destruct();

	cout << "Successful exit." << endl;
    #endif
    return 0;

}