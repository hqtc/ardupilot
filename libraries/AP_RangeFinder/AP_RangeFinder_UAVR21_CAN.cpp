#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_UAVR21_CAN.h"

#if HAL_MAX_CAN_PROTOCOL_DRIVERS

extern const AP_HAL::HAL& hal;
const AP_Param::GroupInfo AP_RangeFinder_UAVR21_CAN::var_info[] = {

    // @Param: RECV_ID
    // @DisplayName: CAN receive ID
    // @Description: The receive ID of the CAN frames. A value of zero means all IDs are accepted.
    // @Range: 0 65535
    // @User: Advanced
    AP_GROUPINFO("RECV_ID", 10, AP_RangeFinder_UAVR21_CAN, receive_id, 0),

    // @Param: SNR_MIN
    // @DisplayName: Minimum signal strength
    // @Description: Minimum signal strength (SNR) to accept distance
    // @Range: 0 65535
    // @User: Advanced
    AP_GROUPINFO("SNR_MIN", 11, AP_RangeFinder_UAVR21_CAN, snr_min, 0),

    AP_GROUPEND
};


/*
  constructor
 */
AP_RangeFinder_UAVR21_CAN::AP_RangeFinder_UAVR21_CAN(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
    CANSensor("UAVR21"),
    AP_RangeFinder_Backend(_state, _params)
{
    AP_Param::setup_object_defaults(this, var_info);
    register_driver(AP_CANManager::Driver_Type_UAVR21);
    state.var_info = var_info;
}

// update state
void AP_RangeFinder_UAVR21_CAN::update(void)
{
    WITH_SEMAPHORE(_sem);
    const uint32_t now = AP_HAL::millis();
    // hal.console->printf("enter driver' update code\n"); // have tested, entered
    if (_distance_count == 0 && now - state.last_reading_ms > 500) {
        // no new data.
        set_status(RangeFinder::Status::NoData);
    } else if (_distance_count != 0) {

        state.distance_cm = (_distance_sum_cm / _distance_count);
        // gcs().send_text(MAV_SEVERITY_CRITICAL, "distance is %d\n", state.distance_cm);

        state.last_reading_ms = AP_HAL::millis();
        _distance_sum_cm = 0;
        _distance_count = 0;
        update_status();
    }
}

// handler for incoming frames. These come in at 100Hz
void AP_RangeFinder_UAVR21_CAN::handle_frame(AP_HAL::CANFrame &frame)
{
    WITH_SEMAPHORE(_sem);
    const uint16_t id = frame.id & AP_HAL::CANFrame::MaskExtID;

    if (receive_id != 0 && id != uint16_t(receive_id.get())) {
        // incorrect receive ID
        return;
    }
    if (last_recv_id != -1 && id != last_recv_id) {
        // changing ID
        return;
    }

    last_recv_id = id;
    const uint16_t dist_cm = (frame.data[2]<<8)+frame.data[3]; // get distance form frame
    
    _distance_sum_cm += dist_cm;
    _distance_count++;
}

#endif // HAL_MAX_CAN_PROTOCOL_DRIVERS
