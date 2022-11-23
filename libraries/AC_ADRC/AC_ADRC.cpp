/// @file	AC_ADRC.cpp
/// @brief	Generic ADRC algorithm

#include <AP_Math/AP_Math.h>
#include "AC_ADRC.h"

const AP_Param::GroupInfo AC_ADRC::var_info[] = {
    // @Param: R
    // @DisplayName: ADRC r
    // @Description: TD parameter for ADRC custom controller backend
    // @User: Advanced
    AP_GROUPINFO("R", 1, AC_ADRC, r, 5.00f),

    // @Param: H0
    // @DisplayName: ADRC h0
    // @Description: TD parameter for ADRC custom controller backend
    // @User: Advanced
    AP_GROUPINFO("H0", 2, AC_ADRC, h0, 0.025f),

    // @Param: ALPHA01
    // @DisplayName: ADRC alpha01
    // @Description: NLSEF parameter for ADRC custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ALPHA01", 3, AC_ADRC, alpha01, 0.75f),

    // @Param: ALPHA02
    // @DisplayName: ADRC alpha02
    // @Description: NLSEF parameter for ADRC custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ALPHA02", 4, AC_ADRC, alpha02, 1.50f),

    // @Param: DELTA
    // @DisplayName: ADRC delta
    // @Description: NLSEF parameter for ADRC custom controller backend
    // @User: Advanced
    AP_GROUPINFO("DELTA", 5, AC_ADRC, delta, 0.0125f),

    // @Param: WO
    // @DisplayName: ADRC wo
    // @Description: ESO parameter for ADRC custom controller backend
    // @User: Advanced
    AP_GROUPINFO("WO", 6, AC_ADRC, wo, 10.00f),

    // @Param: WC
    // @DisplayName: ADRC wc
    // @Description: ADRC parameter for ADRC custom controller backend
    // @User: Advanced
    AP_GROUPINFO("WC", 7, AC_ADRC, wc, 0.10f),

    // @Param: B0
    // @DisplayName: ADRC b0
    // @Description: ADRC parameter for ADRC custom controller backend
    // @User: Advanced
    AP_GROUPINFO("B0", 8, AC_ADRC, b0, 0.20f),

    // @Param: H
    // @DisplayName: ADRC h
    // @Description: ADRC parameter for ADRC custom controller backend
    // @User: Advanced
    AP_GROUPINFO("H", 9, AC_ADRC, h, 0.0025f),

    AP_GROUPEND
};

// Constructor
AC_ADRC::AC_ADRC(float initial_r, float initial_h0, float initial_alpha01, float initial_alpha02, float initial_delta, float initial_wo,
    float initial_wc, float initial_b0, float initial_h)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);
    r.set_and_default(initial_r);
    h0.set_and_default(initial_h0);
    alpha01.set_and_default(initial_alpha01);
    alpha02.set_and_default(initial_alpha02);
    delta.set_and_default(initial_delta);
    wo.set_and_default(initial_wo);
    wc.set_and_default(initial_wc);
    b0.set_and_default(initial_b0);
    h.set_and_default(initial_h);
}

/// Overload the function call operator to permit easy initialisation
void AC_ADRC::operator()(float r_val, float h0_val, float alpha01_val, float alpha02_val, float delta_val, float wo_val, float wc_val, float b0_val, float h_val)
{
    r.set(r_val);
    h0.set(h0_val);
    alpha01.set(alpha01_val);
    alpha02.set(alpha02_val);
    delta.set(delta_val);
    wo.set(wo_val);
    wc.set(wc_val);
    b0.set(b0_val);
    h.set(h_val);
}

// v:设定值    y:输出值   返回值u:控制量
float AC_ADRC::update_all(float v, float y)
{
    // ------实现过程------
    // TD跟踪微分器->过渡过程
    v1 = v1 + h * v2;
    v2 = v2 + h * fst(v1 - v, v2);

    // ESO观测器->估计状态和总扰动
    float e = z1 - y;
    z1 = z1 + h * (z2 - beta01 * e);
    z2 = z2 + h * (z3 - beta02 * fal(e, alpha01, delta) + b0 * u);     // fal函数的作用和解释在"非光滑反馈"部分
    z3 = z3 + h * (- beta03 * fal(e, alpha02, delta));

    // NLSEF和ADRC->计算误差反馈控制量u0和控制量u
    e1 = v1 - z1;
    e2 = v2 - z2;
    u0 = kp * fal(e1, alpha01, delta) + kd * fal(e2, alpha02, delta);   // NLSEF
    u = u0 - (z3 / b0);

    return u;
}

void AC_ADRC::reset_eso(float measurement)
{
    z1 = measurement;
    z2 = 0.0;
    z3 = 0.0;
}

// fst(x1, x2, r, h0): 快速最优控制综合函数,此处需要的后两个参数已在全局定义
float AC_ADRC::fst(float x1, float x2)
{
    float d = r * h0;
    float d0 = h0 * d;
    float y = x1 + h0 * x2;
    float a0 = powf(d * d + 8 * r * fabs(y), 0.5);
    float a = 0; 
    if(fabs(y) > d0) {
        a = x2 + (a0 -d) / 2 * sign(y);
    } else {
        a = x2 + y / h0;
    }

    if(fabs(a) > d) {
        return -r * sign(a);
    } else {
        return -r * a / d;
    }
}

// 一种"非线性组合形式"
float AC_ADRC::fal(float e, float alpha, float del)
{
    if(fabs(e) > del) {
        return powf(fabs(e), alpha) * sign(e);
    } else {
        return e / powf(del, (1-alpha));
    }
}

// 符号函数
float AC_ADRC::sign(float e)
{
    if(e > 0) {
        return 1;
    } else if(e < 0) {
        return -1;
    } else {
        return 0;
    }
}
