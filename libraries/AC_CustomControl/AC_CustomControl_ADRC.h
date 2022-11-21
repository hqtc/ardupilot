#pragma once

#include "AC_CustomControl_Backend.h"

#ifndef CUSTOMCONTROL_ADRC_ENABLED
    #define CUSTOMCONTROL_ADRC_ENABLED AP_CUSTOMCONTROL_ENABLED
#endif

#if CUSTOMCONTROL_ADRC_ENABLED

class AC_CustomControl_ADRC : public AC_CustomControl_Backend {
public:
    AC_CustomControl_ADRC(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl_Multi*& att_control, AP_MotorsMulticopter*& motors, float dt);


    Vector3f update(void) override;
    void reset(void) override;

    float adrc_update(float v, float y);
    float fst(float x1, float x2);
    float fal(float e, float alpha, float del);
    float sign(float e);

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // controller related variable

    // TD
    AP_Float r;
    AP_Float h0;

    // NLSEF
    AP_Float alpha01;
    AP_Float alpha02;
    AP_Float delta;

    // ESO
    AP_Float wo;

    // ADRC
    AP_Float wc;
    AP_Float b0;
    AP_Float h;     // time step
};

#endif
