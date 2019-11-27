#include "GeneralStateProvider.hpp"

GeneralStateProvider::GeneralStateProvider(AttitudeProvider* t_attitude_provider, PositioningProvider* t_position_provider,
                                            HeadingProvider* t_heading_provider) {
    _attitude_provider = t_attitude_provider;
    _position_provider = t_position_provider;
    _heading_provider = t_heading_provider;

}

GeneralStateProvider::~GeneralStateProvider() {

}

Vector3D GeneralStateProvider::getProcessVariable(control_system t_control_system){

    Vector3D process_variable;

    switch (t_control_system)
    {
    case control_system::x:
    {
        process_variable.x = _position_provider->getPosition().x;
        process_variable.y = 0.0; //TODO X_dot
        process_variable.z = 0.0; //TODO X_dot_dot
        break;
    }
    case control_system::y:
    {
        process_variable.x = _position_provider->getPosition().y;
        process_variable.y = 0.0; //TODO Y_dot
        process_variable.z = 0.0; //TODO Y_dot_dot
        break;
    }
    case control_system::z:
    {
        process_variable.x = _position_provider->getPosition().z;
        process_variable.y = 0.0; //TODO Z_dot
        process_variable.z = 0.0; //TODO Z_dot_dot
        break;
    }
    case control_system::roll:
    {
        process_variable.x = _attitude_provider->getAttitude().x;
        process_variable.y = 0.0; //TODO roll_dot
        process_variable.z = 0.0; //TODO roll_dot_dot
        break;
    }
    case control_system::pitch:
    {
        process_variable.x = _attitude_provider->getAttitude().y;
        process_variable.y = 0.0; //TODO pitch_dot
        process_variable.z = 0.0; //TODO pitch_dot_dot
        break;
    }
    case control_system::yaw:
    {
        process_variable.x = _attitude_provider->getAttitude().z;
        process_variable.y = 0.0; //TODO yaw_dot
        process_variable.z = 0.0; //TODO yaw_dot_dot
        break;
    }
    default:
        break;
    }

    return process_variable;

}