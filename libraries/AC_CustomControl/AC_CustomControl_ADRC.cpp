#include "AC_CustomControl_ADRC.h"

#if CUSTOMCONTROL_ADRC_ENABLED

#include <GCS_MAVLink/GCS.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl_ADRC::var_info[] = {
    // @Param: R
    // @DisplayName: ADRC r
    // @Description: TD parameter for ADRC custom controller backend
    // @User: Advanced
    AP_GROUPINFO("R", 1, AC_CustomControl_ADRC, r, 10.00f),

    // @Param: H0
    // @DisplayName: ADRC h0
    // @Description: TD parameter for ADRC custom controller backend
    // @User: Advanced
    AP_GROUPINFO("H0", 2, AC_CustomControl_ADRC, h0, 0.01f),

    // @Param: ALPHA01
    // @DisplayName: ADRC alpha01
    // @Description: NLSEF parameter for ADRC custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ALPHA01", 3, AC_CustomControl_ADRC, alpha01, 0.00f),

    // @Param: ALPHA02
    // @DisplayName: ADRC alpha02
    // @Description: NLSEF parameter for ADRC custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ALPHA02", 4, AC_CustomControl_ADRC, alpha02, 0.00f),

    // @Param: DELTA
    // @DisplayName: ADRC delta
    // @Description: NLSEF parameter for ADRC custom controller backend
    // @User: Advanced
    AP_GROUPINFO("DELTA", 5, AC_CustomControl_ADRC, delta, 0.00f),

    // @Param: WO
    // @DisplayName: ADRC wo
    // @Description: ESO parameter for ADRC custom controller backend
    // @User: Advanced
    AP_GROUPINFO("WO", 6, AC_CustomControl_ADRC, wo, 0.00f),

    // @Param: WC
    // @DisplayName: ADRC wc
    // @Description: ADRC parameter for ADRC custom controller backend
    // @User: Advanced
    AP_GROUPINFO("WC", 7, AC_CustomControl_ADRC, wc, 0.00f),

    // @Param: B0
    // @DisplayName: ADRC b0
    // @Description: ADRC parameter for ADRC custom controller backend
    // @User: Advanced
    AP_GROUPINFO("B0", 8, AC_CustomControl_ADRC, b0, 0.00f),

    AP_GROUPEND
};

// initialize in the constructor
AC_CustomControl_ADRC::AC_CustomControl_ADRC(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl_Multi*& att_control, AP_MotorsMulticopter*& motors, float dt) :
    AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt),
    r(10.00f),
    h0(0.01f),
    alpha01(0.00f),
    alpha02(0.00f),
    delta(0.00f),
    wo(0.00f),
    wc(0.00f),
    b0(0.00f)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// update controller
// return roll, pitch, yaw controller output
Vector3f AC_CustomControl_ADRC::update(void)
{
    // reset controller based on spool state
    switch (_motors->get_spool_state()) {
        case AP_Motors::SpoolState::SHUT_DOWN:
        case AP_Motors::SpoolState::GROUND_IDLE:
            // We are still at the ground. Reset custom controller to avoid
            // build up, ex: integrator
            reset();
            break;

        case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        case AP_Motors::SpoolState::SPOOLING_UP:
        case AP_Motors::SpoolState::SPOOLING_DOWN:
            // we are off the ground
            break;
    }

    // run custom controller after here
     Quaternion attitude_body, attitude_target;
    _ahrs->get_quat_body_to_ned(attitude_body);

    attitude_target = _att_control->get_attitude_target_quat();
    // This vector represents the angular error to rotate the thrust vector using x and y and heading using z
    Vector3f attitude_error;
    float _thrust_angle, _thrust_error_angle;
    _att_control->thrust_heading_rotation_angles(attitude_target, attitude_body, attitude_error, _thrust_angle, _thrust_error_angle);

    // recalculate ang vel feedforward from attitude target model
    // rotation from the target frame to the body frame
    Quaternion rotation_target_to_body = attitude_body.inverse() * attitude_target;
    // target angle velocity vector in the body frame
    Vector3f ang_vel_body_feedforward = rotation_target_to_body * _att_control->get_attitude_target_ang_vel();

    // run attitude controller
    Vector3f target_rate;
    // target_rate[0] = _p_angle_roll2.kP() * attitude_error.x + ang_vel_body_feedforward[0];
    // target_rate[1] = _p_angle_pitch2.kP() * attitude_error.y + ang_vel_body_feedforward[1];
    // target_rate[2] = _p_angle_yaw2.kP() * attitude_error.z + ang_vel_body_feedforward[2];

    // run rate controller
    Vector3f gyro_latest = _ahrs->get_gyro_latest();
    Vector3f motor_out;
    motor_out.x = adrc_update(target_rate[0], gyro_latest[0]);
    motor_out.y = adrc_update(target_rate[1], gyro_latest[1]);
    motor_out.z = adrc_update(target_rate[2], gyro_latest[2]);
 
    // gcs().send_text(MAV_SEVERITY_INFO, "ADRC custom controller working");

    return motor_out;
}

// reset controller to avoid build up on the ground
// or to provide bumpless transfer from arducopter main controller
void AC_CustomControl_ADRC::reset(void)
{
}

// v:设定值    y:输出值   返回值u:控制量
float AC_CustomControl_ADRC::adrc_update(float v, float y)
{
    // ------定义参数------
    // TD跟踪微分器
    float v1;
    float v2;

    // ESO观测器
    float beta01 = 3 * wo;
    float beta02 = 3 * wo * wo;
    float beta03 = wo * wo * wo;
    float z1 = 0, z2 = 0, z3 = 0;

    // NLSEF非线性误差反馈控制
    float kp = 2 * wc;      // 比例项增益
    float kd = wc * wc;     // 微分项增益
    float u0 = 0;           // u0:误差反馈控制量

    // ADRC控制器
    float h = 0.1;          // h:积分步长
    float e1 = 0, e2 = 0;   // e1,e2:误差
    float u  = 0;

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

// fst(x1, x2, r, h0): 快速最优控制综合函数,此处需要的后两个参数已在全局定义
float AC_CustomControl_ADRC::fst(float x1, float x2)
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
float AC_CustomControl_ADRC::fal(float e, float alpha, float del)
{
    if(fabs(e) > del) {
        return powf(fabs(e), alpha) * sign(e);
    } else {
        return e / powf(del, (1-alpha));
    }
}

// 符号函数
float AC_CustomControl_ADRC::sign(float e)
{
    if(e > 0) {
        return 1;
    } else if(e == 0) {
        return 0;
    } else {
        return -1;
    }
}

#endif
