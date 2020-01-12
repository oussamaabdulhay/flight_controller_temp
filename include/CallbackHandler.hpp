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

#include <iostream>
#include <iomanip>
#include <list>
#include <string>

using namespace std;
#undef DEBUG_XSENS

#ifdef DEBUG_XSENS
Timer t;
#endif

class CallbackHandler : public XsCallback, public msg_emitter
{
public:
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
		XSensMessage m_output_msg;
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
            orientation_euler.x = euler.roll() * M_PI / 180.0;
            orientation_euler.y = euler.pitch() * M_PI / 180.0;
            orientation_euler.z = euler.yaw() * M_PI / 180.0;
            m_output_msg.setOrientationEuler(orientation_euler);
        }

        if (t_packet->containsCalibratedGyroscopeData()){

            XsVector gyr = t_packet->calibratedGyroscopeData();
            Vector3D<float> angular_vel;
            angular_vel.x = gyr[0];
            angular_vel.y = gyr[1];
            angular_vel.z = gyr[2];
            m_output_msg.setAngularVelocity(angular_vel);

        }
		#ifdef DEBUG_XSENS
		std::cout << "TIME: " << t.tockMilliSeconds() << "\n";
		t.tick();
		#endif
        
        this->emit_message((DataMessage*) &m_output_msg);
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