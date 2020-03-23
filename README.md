# flight_controller
Linux based flight controller.
On going project at Khalifa University Center for Autonomous Robots.

    ========                                                                               =============
    |      |----->X_Control_System-->RM_X-->Saturation-->Roll_Control_System-------------->|           |
    | USER |----->Y_Control_System-->RM_Y-->Saturation-->Pitch_Control_System------------->| Actuation |
    |      |----->Z_Control_System-------------------------------------------------------->|  System   |
    |      |----->Yaw_Control_System-->Saturation--->YawRate_Control_System--------------->|           |
    ========                                                                               =============
