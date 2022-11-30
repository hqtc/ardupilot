#pragma once

/// @file	AC_PID.h
/// @brief	Generic ADRC algorithm, with EEPROM-backed storage of constants.

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <stdlib.h>
#include <cmath>
#include <Filter/SlewLimiter.h>

class AC_ADRC {
public:

    // Constructor for ADRC
    AC_ADRC(float initial_r, float initial_h0, float initial_alpha01, float initial_alpha02, float initial_delta, float initial_beta01, float initial_beta02, float initial_beta03,
    float initial_kp, float initial_kd, float initial_b0, float initial_h);

    CLASS_NO_COPY(AC_ADRC);

    //  update_all - set target and measured inputs to PID controller and calculate outputs
    float update_all(float v, float y);

    void reset_eso(float measurement);

    float fst(float x1, float x2);
    float fal(float e, float alpha, float del);

    float sign(float e);

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // controller related parameters
    AP_Float r;             // TD
    AP_Float h0;
    AP_Float alpha01;       // NLSEF
    AP_Float alpha02;
    AP_Float delta;
    // AP_Float wo;         // ESO
    AP_Float beta01;
    AP_Float beta02;
    AP_Float beta03;
    // AP_Float wc;         // ADRC
    AP_Float kp;
    AP_Float kd;
    AP_Float b0;
    AP_Float h;             // time step

    // float h;                // time step

    // internal variables
    float v1 = 0;           // TD跟踪微分器
    float v2 = 0;
    
    float z1 = 0;
    float z2 = 0;
    float z3 = 0;
    
    float u0 = 0;           // NLSEF非线性误差反馈控制-u0:误差反馈控制量
    
    float e1 = 0;           // ADRC控制器-e1,e2:误差
    float e2 = 0;
    float u = 0;
};
