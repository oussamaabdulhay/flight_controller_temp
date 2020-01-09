
//  ==> COPYRIGHT (C) 2019 XSENS TECHNOLOGIES OR SUBSIDIARIES WORLDWIDE <==
//  WARNING: COPYRIGHT (C) 2019 XSENS TECHNOLOGIES OR SUBSIDIARIES WORLDWIDE. ALL RIGHTS RESERVED.
//  THIS FILE AND THE SOURCE CODE IT CONTAINS (AND/OR THE BINARY CODE FILES FOUND IN THE SAME
//  FOLDER THAT CONTAINS THIS FILE) AND ALL RELATED SOFTWARE (COLLECTIVELY, "CODE") ARE SUBJECT
//  TO AN END USER LICENSE AGREEMENT ("AGREEMENT") BETWEEN XSENS AS LICENSOR AND THE AUTHORIZED
//  LICENSEE UNDER THE AGREEMENT. THE CODE MUST BE USED SOLELY WITH XSENS PRODUCTS INCORPORATED
//  INTO LICENSEE PRODUCTS IN ACCORDANCE WITH THE AGREEMENT. ANY USE, MODIFICATION, COPYING OR
//  DISTRIBUTION OF THE CODE IS STRICTLY PROHIBITED UNLESS EXPRESSLY AUTHORIZED BY THE AGREEMENT.
//  IF YOU ARE NOT AN AUTHORIZED USER OF THE CODE IN ACCORDANCE WITH THE AGREEMENT, YOU MUST STOP
//  USING OR VIEWING THE CODE NOW, REMOVE ANY COPIES OF THE CODE FROM YOUR COMPUTER AND NOTIFY
//  XSENS IMMEDIATELY BY EMAIL TO INFO@XSENS.COM. ANY COPIES OR DERIVATIVES OF THE CODE (IN WHOLE
//  OR IN PART) IN SOURCE CODE FORM THAT ARE PERMITTED BY THE AGREEMENT MUST RETAIN THE ABOVE
//  COPYRIGHT NOTICE AND THIS PARAGRAPH IN ITS ENTIRETY, AS REQUIRED BY THE AGREEMENT.
//  
//  THIS SOFTWARE CAN CONTAIN OPEN SOURCE COMPONENTS WHICH CAN BE SUBJECT TO 
//  THE FOLLOWING GENERAL PUBLIC LICENSES:
//  ==> Qt GNU LGPL version 3: http://doc.qt.io/qt-5/lgpl.html <==
//  ==> LAPACK BSD License:  http://www.netlib.org/lapack/LICENSE.txt <==
//  ==> StackWalker 3-Clause BSD License: https://github.com/JochenKalmbach/StackWalker/blob/master/LICENSE <==
//  ==> Icon Creative Commons 3.0: https://creativecommons.org/licenses/by/3.0/legalcode <==
//  

#include "xdainterface.h"

#include <xscontroller/xsscanner.h>
#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <fstream>
#include "messagepublishers/angularvelocitypublisher.h"
#include "messagepublishers/orientationpublisher.h"

std::ofstream write_data1("/home/pedrohrpbs/catkin_ws_NAVIO//orientation_control_data_pure.txt"); 

XdaInterface::XdaInterface(std::string t_port_name, int t_baudrate)
	: m_device(nullptr)
{
	timer.tick();
	m_port_name = &t_port_name;
	m_baudrate = t_baudrate;
	std::cout << "Creating XsControl object..." << std::endl;
	m_control = XsControl::construct();
	assert(m_control != 0);
}

XdaInterface::~XdaInterface()
{
	close();
	m_control->destruct();
}

void XdaInterface::spinFor(std::chrono::milliseconds timeout)
{
	XsDataPacket Packet = m_xdaCallback.next(timeout);

	if (!Packet.empty())
	{
		for (auto &cb : m_callbacks)
		{
			DataMessage* msg = cb->operator()(Packet);

			if(msg->getType() == msg_type::VECTOR3D){
				Vector3D<float> ang_vel = ((Vector3DMessage*)msg)->getData();
				m_output_msg.setAngularVelocity(ang_vel);
			}else if(msg->getType() == msg_type::QUATERNION){
				Quaternion ori = ((QuaternionMessage*)msg)->getData();
				write_data1 << ori.x << ", " << timer.tockMilliSeconds() << "\n";
				timer.tick();
				m_output_msg.setOrientation(ori);
			}
		}

		emit_message((DataMessage*)&m_output_msg);
	}
}

void XdaInterface::registerPublishers()
{
	bool should_publish;

	// if (ros::param::get("~pub_imu", should_publish) && should_publish)
	// {
	// 	registerCallback(new ImuPublisher(node));
	// }
	if (1)
	{
		registerCallback(new OrientationPublisher());
	}
	// if (ros::param::get("~pub_acceleration", should_publish) && should_publish)
	// {
	// 	registerCallback(new AccelerationPublisher(node));
	// }
	if (1)
	{
		registerCallback(new AngularVelocityPublisher());
	}
	// if (ros::param::get("~pub_mag", should_publish) && should_publish)
	// {
	// 	registerCallback(new MagneticFieldPublisher(node));
	// }
	// if (ros::param::get("~pub_dq", should_publish) && should_publish)
	// {
	// 	registerCallback(new OrientationIncrementsPublisher(node));
	// }
	// if (ros::param::get("~pub_dv", should_publish) && should_publish)
	// {
	// 	registerCallback(new VelocityIncrementPublisher(node));
	// }
	// if (ros::param::get("~pub_sampletime", should_publish) && should_publish)
	// {
	// 	registerCallback(new TimeReferencePublisher(node));
	// }
	// if (ros::param::get("~pub_temperature", should_publish) && should_publish)
	// {
	// 	registerCallback(new TemperaturePublisher(node));
	// }
	// if (ros::param::get("~pub_pressure", should_publish) && should_publish)
	// {
	// 	registerCallback(new PressurePublisher(node));
	// }
	// if (ros::param::get("~pub_gnss", should_publish) && should_publish)
	// {
	// 	registerCallback(new GnssPublisher(node));
	// }
	// if (ros::param::get("~pub_twist", should_publish) && should_publish)
	// {
	// 	registerCallback(new TwistPublisher(node));
	// }
	// if (ros::param::get("~pub_free_acceleration", should_publish) && should_publish)
	// {
	// 	registerCallback(new FreeAccelerationPublisher(node));
	// }
	// if (ros::param::getCached("~pub_transform", should_publish) && should_publish)
	// {
	// 	registerCallback(new TransformPublisher(node));
	// }
}

bool XdaInterface::connectDevice()
{
	XsPortInfo mtPort;

	if (m_port_name != nullptr)
	{
		mtPort = XsPortInfo(*m_port_name, XsBaud_numericToRate(m_baudrate));
	}
	else
	{
		std::cout << "Scanning for devices..." << std::endl;
		XsPortInfoArray portInfoArray = XsScanner::scanPorts();

		for (auto const &portInfo : portInfoArray)
		{
			if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
			{
				mtPort = portInfo;
				break;
			}
		}
	}

	if (mtPort.empty()){
		return handleError("No MTi device found");
	}
		
	printf("Found a device with ID: %s @ port: %s, baudrate: %d", mtPort.deviceId().toString().toStdString().c_str(), mtPort.portName().toStdString().c_str(), mtPort.baudrate());
	
	printf("Opening port...");
	if (!m_control->openPort(mtPort.portName().toStdString(), mtPort.baudrate()))
		return handleError("Could not open port");

	m_device = m_control->device(mtPort.deviceId());
	assert(m_device != 0);

	printf("Device: %s, with ID: %s opened.", m_device->productCode().toStdString().c_str(), m_device->deviceId().toString().c_str());

	m_device->addCallbackHandler(&m_xdaCallback);

	return true;
}

bool XdaInterface::prepare()
{
	assert(m_device != 0);

	if (!m_device->gotoConfig())
		return handleError("Could not go to config");

	// read EMTS and device config stored in .mtb file header.
	if (!m_device->readEmtsAndDeviceConfiguration())
		return handleError("Could not read device configuration");

	if (!m_device->gotoMeasurement())
		return handleError("Could not put device into measurement mode");

	// std::string log_file;
	// if (ros::param::get("~log_file", log_file))
	// {
	// 	if (m_device->createLogFile(log_file) != XRV_OK)
	// 		return handleError(std::string("Failed to create a log file! (%s)") + log_file);
	// 	else
	// 		ROS_INFO("Created a log file: %s", log_file.c_str());

	// 	if (!m_device->startRecording())
	// 		return handleError("Could not start recording");
	// }

	return true;
}

void XdaInterface::close()
{
	if (m_device != nullptr)
	{
		m_device->stopRecording();
		m_device->closeLogFile();
		m_device->removeCallbackHandler(&m_xdaCallback);
	}
	m_control->closePort(m_port);
}

void XdaInterface::registerCallback(PacketCallback *cb)
{
	m_callbacks.push_back(cb);
	std::cout << "adding register" << std::endl;
}

bool XdaInterface::handleError(std::string error)
{
	printf("%s", error.c_str());
	return false;
}
