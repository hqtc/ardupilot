#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AC_ADRC/AC_ADRC.h>

#include "AC_CustomControl_Backend.h"

#ifndef CUSTOMCONTROL_ADRC_ENABLED
    #define CUSTOMCONTROL_ADRC_ENABLED AP_CUSTOMCONTROL_ENABLED
#endif

#if CUSTOMCONTROL_ADRC_ENABLED

class AC_CustomControl_ADRC : public AC_CustomControl_Backend {
public:
    AC_CustomControl_ADRC(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl_Multi*& att_control, AP_MotorsMulticopter*& motors, float dt);

    // run lowest level body-frame rate controller and send outputs to the motors
    Vector3f update(void) override;
    void reset(void) override;

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // put controller related variable here
    
    // rate ADRC controller  objects
    AC_ADRC _adrc_rate_roll;
    AC_ADRC _adrc_rate_pitch;
    AC_ADRC _adrc_rate_yaw;
};

#endif
