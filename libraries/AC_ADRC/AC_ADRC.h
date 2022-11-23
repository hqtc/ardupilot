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
    AC_ADRC(float initial_r, float initial_h0, float initial_alpha01, float initial_alpha02, float initial_delta, float initial_wo,
            float initial_wc, float initial_b0, float initial_h);

    CLASS_NO_COPY(AC_ADRC);

    //  update_all - set target and measured inputs to PID controller and calculate outputs
    float update_all(float v, float y);

    void reset_eso(float measurement);

    float fst(float x1, float x2);
    float fal(float e, float alpha, float del);

    float sign(float e);

    /// operator function call for easy initialisation
    void operator()(float r_val, float h0_val, float alpha01_val, float alpha02_val, float delta_val, float wo_val, float wc_val, float b0_val, float h_val);

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // controller related parameters
    AP_Float r;             // TD
    AP_Float h0;
    AP_Float alpha01;       // NLSEF
    AP_Float alpha02;
    AP_Float delta;
    AP_Float wo;            // ESO
    AP_Float wc;            // ADRC
    AP_Float b0;
    AP_Float h;             // time step

    // internal variables
    float v1 ;              // TD跟踪微分器
    float v2 ;
    
    float beta01 = wo;      // ESO观测器
    float beta02 = 3 * wo * wo;
    float beta03 = wo * wo * wo;
    float z1 = 0;
    float z2 = 0;
    float z3 = 0;
    
    float kp = 2 * wc;      // NLSEF非线性误差反馈控制-比例项增益
    float kd = wc * wc;     // NLSEF非线性误差反馈控制-微分项增益
    float u0 = 0;           // NLSEF非线性误差反馈控制-u0:误差反馈控制量
    
    float e1 = 0;           // // ADRC控制器-e1,e2:误差
    float e2 = 0;
    float u = 0;
};
