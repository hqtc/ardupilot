#include "AC_CustomControl_ADRC.h"

#if CUSTOMCONTROL_ADRC_ENABLED

#include <GCS_MAVLink/GCS.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl_ADRC::var_info[] = {
    // @Param: RAT_RLL_LM
    // @DisplayName: ADRC roll axis control output limit
    // @User: Advanced
    AP_SUBGROUPINFO(_adrc_rate_roll, "RAT_R_", 1, AC_CustomControl_ADRC, AC_ADRC),

    // @Param: RAT_PIT_LM
    // @DisplayName: ADRC pitch axis control output limit
    // @User: Advanced
    AP_SUBGROUPINFO(_adrc_rate_pitch, "RAT_P_", 2, AC_CustomControl_ADRC, AC_ADRC),

    // @Param: RAT_YAW_LM
    // @DisplayName: ADRC yaw axis control output limit
    // @User: Advanced
    AP_SUBGROUPINFO(_adrc_rate_yaw, "RAT_Y_", 3, AC_CustomControl_ADRC, AC_ADRC),

    AP_GROUPEND
};

// initialize in the constructor
AC_CustomControl_ADRC::AC_CustomControl_ADRC(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl_Multi*& att_control, AP_MotorsMulticopter*& motors, float dt) :
    AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt),
    _adrc_rate_roll(100.00f, 0.0125f, 0.5f, 0.25f, 0.0025f, 10.00f, 200.00f, 30.00f, 10.00f, 200.00f, 0.20f, dt),
    _adrc_rate_pitch(100.00f, 0.0125f, 0.5f, 0.25f, 0.0025f, 10.00f, 200.00f, 30.00f, 10.00f, 200.00f, 0.20f, dt),
    _adrc_rate_yaw(100.00f, 0.0125f, 0.5f, 0.25f, 0.0025f, 10.00f, 200.00f, 30.00f, 10.00f, 200.00f, 0.20f, dt)
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

    Vector3f rate_target = _att_control->rate_bf_targets();
    Vector3f gyro_latest = _ahrs->get_gyro_latest();

    Vector3f motor_out;

    motor_out.x = _adrc_rate_roll.update_all(rate_target.x, gyro_latest.x);
    motor_out.y = _adrc_rate_pitch.update_all(rate_target.y, gyro_latest.y);
    motor_out.z = _adrc_rate_yaw.update_all(rate_target.z, gyro_latest.z);

    // gcs().send_text(MAV_SEVERITY_INFO, "ADRC custom controller working");

    return motor_out;
}

// reset controller to avoid build up on the ground
// or to provide bumpless transfer from arducopter main controller
void AC_CustomControl_ADRC::reset(void)
{
    Vector3f gyro_latest = _ahrs->get_gyro_latest();
    _adrc_rate_roll.reset_eso(gyro_latest.x);
    _adrc_rate_pitch.reset_eso(gyro_latest.y);
    _adrc_rate_yaw.reset_eso(gyro_latest.z);
}

#endif
