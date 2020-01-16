#pragma once
#include "PositioningProvider.hpp"
#include "AttitudeProvider.hpp"
#include "HeadingProvider.hpp"
#include "VelocityProvider.hpp"
#include "AccelerationProvider.hpp"
#include "X_PVProvider.hpp"
#include "Y_PVProvider.hpp"
#include "Z_PVProvider.hpp"
#include "Roll_PVProvider.hpp"
#include "Pitch_PVProvider.hpp"
#include "Yaw_PVProvider.hpp"
#include "YawRate_PVProvider.hpp"

class MotionCapture :   public msg_receiver, 
                        public X_PVProvider,
                        public Y_PVProvider,
                        public Z_PVProvider,
                        public Roll_PVProvider,
                        public Pitch_PVProvider,
                        public Yaw_PVProvider,
                        public YawRate_PVProvider{

    public:
        virtual PositionMsg getPosition() = 0;
        virtual VelocityMsg getVelocity() = 0;
        virtual AccelerationMsg getAcceleration() = 0;
        virtual AttitudeMsg getAttitude() = 0;
        virtual HeadingMsg getHeading() = 0;
        virtual Vector3D<float> getBodyRate() = 0;
        //BodyRateProvider* getBodyRateProvider() { return NULL; }

        virtual void receive_msg_data(DataMessage* t_msg) = 0;

        MotionCapture();
        ~MotionCapture();
};