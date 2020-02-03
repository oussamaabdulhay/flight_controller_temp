#pragma once
#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xscontroller/xsscanner.h>
#include <xstypes/xsoutputconfigurationarray.h>
#include <xstypes/xsdatapacket.h>
#include <xstypes/xstime.h>
#include <xscommon/xsens_mutex.h>
#include "MsgEmitter.hpp"
#include "XSensMessage.hpp"
#include "Timer.hpp"
#include "Vector3DMessage.hpp"
#include <iostream>
#include <iomanip>
#include <list>
#include <string>
#include "Global2Inertial.hpp"
#include "PVConcatenator.hpp"

using namespace std;
#undef DEBUG_XSENS

#ifdef DEBUG_XSENS
Timer t;
#endif

class CallbackHandler : public XsCallback, public msg_emitter
{
public:
enum unicast_addresses {broadcast,unicast_XSens_translation,unicast_XSens_orientation};
	CallbackHandler(size_t maxBufferSize = 1)
		: m_maxNumberOfPacketsInBuffer(maxBufferSize)
		, m_numberOfPacketsInBuffer(0)
	{
		#ifdef DEBUG_XSENS
		t.tick();
		#endif
	}

	virtual ~CallbackHandler() throw()
	{
	}

	bool packetAvailable() const
	{
		xsens::Lock locky(&m_mutex);
		return m_numberOfPacketsInBuffer > 0;
	}

	XsDataPacket getNextPacket()
	{
		assert(packetAvailable());
		xsens::Lock locky(&m_mutex);
		XsDataPacket oldestPacket(m_packetBuffer.front());
		m_packetBuffer.pop_front();
		--m_numberOfPacketsInBuffer;
		return oldestPacket;
	}

protected:
	void onLiveDataAvailable(XsDevice*, const XsDataPacket* t_packet) override
	{
		Vector3DMessage pv_msg;
		Vector3DMessage pv_dot_msg;
		Vector3DMessage position_msg;
		xsens::Lock locky(&m_mutex);
		assert(t_packet != 0);
		while (m_numberOfPacketsInBuffer >= m_maxNumberOfPacketsInBuffer)
			(void)getNextPacket();

		m_packetBuffer.push_back(*t_packet);

        //**************************************************************
        assert(packetAvailable());

		if (t_packet->containsOrientation())
		{
            XsEuler euler = t_packet->orientationEuler();
            Vector3D<float> orientation_euler;
            orientation_euler.x = euler.pitch() * M_PI / 180.0;
            orientation_euler.y = -1 * euler.roll() * M_PI / 180.0; //Arranging the frames to match with the drone's
            orientation_euler.z = euler.yaw() * M_PI / 180.0;
            pv_msg.setVector3DMessage(orientation_euler);
			this->emit_message_unicast((DataMessage*) &pv_msg,(int)CallbackHandler::unicast_addresses::unicast_XSens_orientation, (int)PVConcatenator::receiving_channels::ch_pv);
        }

        if (t_packet->containsCalibratedGyroscopeData()){

            XsVector gyr = t_packet->calibratedGyroscopeData();
            Vector3D<float> angular_vel;
            angular_vel.x = gyr[1];
            angular_vel.y = -1 * gyr[0];
            angular_vel.z = gyr[2];
            pv_dot_msg.setVector3DMessage(angular_vel);
			//this->emit_message_unicast((DataMessage*) &pv_dot_msg,(int)CallbackHandler::unicast_addresses::unicast_XSens_orientation, (int)PVConcatenator::receiving_channels::ch_pv_dot);

        }
		
		if(t_packet->containsPositionLLA()){

			XsVector pos = t_packet->positionLLA();
            Vector3D<double> position;
            position.x = pos[0];
            position.y = pos[1];
            position.z = pos[2];
            position_msg.setVector3DMessage(position);
			this->emit_message_unicast(&position_msg,(int)CallbackHandler::unicast_addresses::unicast_XSens_translation,(int)Global2Inertial::receiving_channels::ch_XSens_pos);
		}
		#ifdef DEBUG_XSENS
		std::cout << "TIME: " << t.tockMicroSeconds() << "\n";
		t.tick();
		#endif
        
        
        //**************************************************************
		++m_numberOfPacketsInBuffer;
		assert(m_numberOfPacketsInBuffer <= m_maxNumberOfPacketsInBuffer);
	}
private:
	mutable xsens::Mutex m_mutex;
	size_t m_maxNumberOfPacketsInBuffer;
	size_t m_numberOfPacketsInBuffer;
	list<XsDataPacket> m_packetBuffer;
};