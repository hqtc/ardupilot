/// @file	AC_ADRC.cpp
/// @brief	Generic ADRC algorithm

#include <AP_Math/AP_Math.h>
#include "AC_ADRC.h"

const AP_Param::GroupInfo AC_ADRC::var_info[] = {
    // @Param: R
    // @DisplayName: ADRC r
    // @Description: TD parameter for ADRC custom controller backend
    // @User: Advanced
    AP_GROUPINFO("R", 1, AC_ADRC, r, 300.00f),

    // @Param: H0
    // @DisplayName: ADRC h0
    // @Description: TD parameter for ADRC custom controller backend
    // @User: Advanced
    AP_GROUPINFO("H0", 2, AC_ADRC, h0, 0.01f),

    // @Param: ALPHA01
    // @DisplayName: ADRC alpha01
    // @Description: NLSEF parameter for ADRC custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ALPHA1", 3, AC_ADRC, alpha01, 0.50f),

    // @Param: ALPHA02
    // @DisplayName: ADRC alpha02
    // @Description: NLSEF parameter for ADRC custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ALPHA2", 4, AC_ADRC, alpha02, 0.25f),

    // @Param: DELTA
    // @DisplayName: ADRC delta
    // @Description: NLSEF parameter for ADRC custom controller backend
    // @User: Advanced
    AP_GROUPINFO("DELTA", 5, AC_ADRC, delta, 20.00f),

    // // @Param: WO
    // // @DisplayName: ADRC wo
    // // @Description: ESO parameter for ADRC custom controller backend
    // // @User: Advanced
    // AP_GROUPINFO("WO", 6, AC_ADRC, wo, 10.00f),

    // // @Param: WC
    // // @DisplayName: ADRC wc
    // // @Description: ADRC parameter for ADRC custom controller backend
    // // @User: Advanced
    // AP_GROUPINFO("WC", 7, AC_ADRC, wc, 0.10f),

    // @Param: BETA01
    // @DisplayName: ADRC beta01
    // @Description: ESO parameter for ADRC custom controller backend
    // @User: Advanced
    AP_GROUPINFO("BETA01", 6, AC_ADRC, beta01, 200.00f),

    // @Param: BETA02
    // @DisplayName: ADRC beta02
    // @Description: ESO parameter for ADRC custom controller backend
    // @User: Advanced
    AP_GROUPINFO("BETA02", 7, AC_ADRC, beta02, 2000.00f),

    // @Param: BETA03
    // @DisplayName: ADRC beta03
    // @Description: ESO parameter for ADRC custom controller backend
    // @User: Advanced
    AP_GROUPINFO("BETA03", 8, AC_ADRC, beta03, 100.00f),

    // @Param: KP
    // @DisplayName: ADRC kp
    // @Description: ADRC parameter for ADRC custom controller backend
    // @User: Advanced
    AP_GROUPINFO("KP", 9, AC_ADRC, kp, 0.325f),

    // @Param: KD
    // @DisplayName: ADRC kd
    // @Description: ADRC parameter for ADRC custom controller backend
    // @User: Advanced
    AP_GROUPINFO("KD", 10, AC_ADRC, kd, 0.0015f),

    // @Param: B0
    // @DisplayName: ADRC b0
    // @Description: ADRC parameter for ADRC custom controller backend
    // @User: Advanced
    AP_GROUPINFO("B0", 11, AC_ADRC, b0, 5.00f),

    // @Param: H
    // @DisplayName: ADRC h
    // @Description: ADRC parameter for ADRC custom controller backend
    // @User: Advanced
    AP_GROUPINFO("H", 12, AC_ADRC, h, 0.005f),

    AP_GROUPEND
};

// Constructor
AC_ADRC::AC_ADRC(float initial_r, float initial_h0, float initial_alpha01, float initial_alpha02, float initial_delta, float initial_beta01, float initial_beta02, float initial_beta03,
    float initial_kp, float initial_kd, float initial_b0, float initial_h)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);
    r.set_and_default(initial_r);
    h0.set_and_default(initial_h0);
    alpha01.set_and_default(initial_alpha01);
    alpha02.set_and_default(initial_alpha02);
    delta.set_and_default(initial_delta);
    beta01.set_and_default(initial_beta01);
    beta02.set_and_default(initial_beta02);
    beta03.set_and_default(initial_beta03);
    kp.set_and_default(initial_kp);
    kd.set_and_default(initial_kd);
    b0.set_and_default(initial_b0);
    h.set_and_default(initial_h);
    // h = dt;
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
    u0 = kp * fal(e1, 0.7, delta) + kd * fal(e2, 1, delta);   // NLSEF
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
