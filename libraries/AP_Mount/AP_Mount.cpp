#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include "AP_Mount.h"
#include "AP_Mount_Backend.h"
#include "AP_Mount_Alexmos.h"
#include "AP_Mount_QHPayload.h"

const AP_Param::GroupInfo AP_Mount::var_info[] = {
    // @Param: _DEFLT_MODE
    // @DisplayName: Mount default operating mode
    // @Description: Mount default operating mode on startup and after control is returned from autopilot
    // @Values: 0:Retracted,1:Neutral,2:MavLink Targeting,3:RC Targeting,4:GPS Point
    // @User: Standard
    AP_GROUPINFO("_DEFLT_MODE", 0, AP_Mount, state._default_mode, MAV_MOUNT_MODE_RC_TARGETING),

    // @Param: _RETRACT_X
    // @DisplayName: Mount roll angle when in retracted position
    // @Description: Mount roll angle when in retracted position
    // @Units: deg
    // @Range: -180.00 179.99
    // @Increment: 1
    // @User: Standard

    // @Param: _RETRACT_Y
    // @DisplayName: Mount tilt/pitch angle when in retracted position
    // @Description: Mount tilt/pitch angle when in retracted position
    // @Units: deg
    // @Range: -180.00 179.99
    // @Increment: 1
    // @User: Standard

    // @Param: _RETRACT_Z
    // @DisplayName: Mount yaw/pan angle when in retracted position
    // @Description: Mount yaw/pan angle when in retracted position
    // @Units: deg
    // @Range: -180.00 179.99
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_RETRACT",    1, AP_Mount, state._retract_angles, 0),

    // @Param: _NEUTRAL_X
    // @DisplayName: Mount roll angle when in neutral position
    // @Description: Mount roll angle when in neutral position
    // @Units: deg
    // @Range: -180.00 179.99
    // @Increment: 1
    // @User: Standard

    // @Param: _NEUTRAL_Y
    // @DisplayName: Mount tilt/pitch angle when in neutral position
    // @Description: Mount tilt/pitch angle when in neutral position
    // @Units: deg
    // @Range: -180.00 179.99
    // @Increment: 1
    // @User: Standard

    // @Param: _NEUTRAL_Z
    // @DisplayName: Mount pan/yaw angle when in neutral position
    // @Description: Mount pan/yaw angle when in neutral position
    // @Units: deg
    // @Range: -180.00 179.99
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_NEUTRAL",    2, AP_Mount, state._neutral_angles, 0),

    // 3 was used for control_angles

    // @Param: _RC_IN_ROLL
    // @DisplayName: roll RC input channel
    // @Description: 0 for none, any other for the RC channel to be used to control roll movements
    // @Values: 0:Disabled,5:RC5,6:RC6,7:RC7,8:RC8,9:RC9,10:RC10,11:RC11,12:RC12
    // @User: Standard
    AP_GROUPINFO("_RC_IN_ROLL",  3, AP_Mount, state._roll_rc_in, 0),

    // @Param: _ANGMIN_ROL
    // @DisplayName: Minimum roll angle
    // @Description: Minimum physical roll angular position of mount.
    // @Units: cdeg
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_ANGMIN_ROL", 4, AP_Mount, state._roll_angle_min, -4500),

    // @Param: _ANGMAX_ROL
    // @DisplayName: Maximum roll angle
    // @Description: Maximum physical roll angular position of the mount
    // @Units: cdeg
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_ANGMAX_ROL", 5, AP_Mount, state._roll_angle_max, 4500),

    // @Param: _RC_IN_TILT
    // @DisplayName: tilt (pitch) RC input channel
    // @Description: 0 for none, any other for the RC channel to be used to control tilt (pitch) movements
    // @Values: 0:Disabled,5:RC5,6:RC6,7:RC7,8:RC8,9:RC9,10:RC10,11:RC11,12:RC12
    // @User: Standard
    AP_GROUPINFO("_RC_IN_TILT",  6, AP_Mount, state._tilt_rc_in,    0),

    // @Param: _ANGMIN_TIL
    // @DisplayName: Minimum tilt angle
    // @Description: Minimum physical tilt (pitch) angular position of mount.
    // @Units: cdeg
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_ANGMIN_TIL", 7, AP_Mount, state._tilt_angle_min, -4500),

    // @Param: _ANGMAX_TIL
    // @DisplayName: Maximum tilt angle
    // @Description: Maximum physical tilt (pitch) angular position of the mount
    // @Units: cdeg
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_ANGMAX_TIL", 8, AP_Mount, state._tilt_angle_max, 4500),

    // @Param: _RC_IN_PAN
    // @DisplayName: pan (yaw) RC input channel
    // @Description: 0 for none, any other for the RC channel to be used to control pan (yaw) movements
    // @Values: 0:Disabled,5:RC5,6:RC6,7:RC7,8:RC8,9:RC9,10:RC10,11:RC11,12:RC12
    // @User: Standard
    AP_GROUPINFO("_RC_IN_PAN",  9, AP_Mount, state._pan_rc_in,       0),

    // @Param: _ANGMIN_PAN
    // @DisplayName: Minimum pan angle
    // @Description: Minimum physical pan (yaw) angular position of mount.
    // @Units: cdeg
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_ANGMIN_PAN",  10, AP_Mount, state._pan_angle_min,  -4500),

    // @Param: _ANGMAX_PAN
    // @DisplayName: Maximum pan angle
    // @Description: Maximum physical pan (yaw) angular position of the mount
    // @Units: cdeg
    // @Range: -18000 17999
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_ANGMAX_PAN",  11, AP_Mount, state._pan_angle_max,  4500),

    // @Param: _JSTICK_SPD
    // @DisplayName: mount joystick speed
    // @Description: 0 for position control, small for low speeds, 100 for max speed. A good general value is 10 which gives a movement speed of 3 degrees per second.
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_JSTICK_SPD",  12, AP_Mount, _joystick_speed, 0),

    ////////////////////////////////////////////////////////////

    AP_GROUPINFO("_TYPE", 15, AP_Mount, state._type, 0),

    AP_GROUPINFO("_T_Zm_CH", 16, AP_Mount, state._Zoom_ch, 6),

    AP_GROUPINFO("_T_Vid_CH", 17, AP_Mount, state._Video_ch, 7),

    AP_GROUPINFO("_T_Rec_CH", 18, AP_Mount, state._Rec_ch, 8),

    AP_GROUPINFO("_T_trck_CH", 19, AP_Mount, state._Track_ch, 10),

    AP_GROUPINFO("_T_Spd_mn", 20, AP_Mount, state._Speed_min, 8),

    AP_GROUPINFO("_T_Spd_mx", 21, AP_Mount, state._Speed_max, 80),

    AP_GROUPINFO("_T_kPmin", 22, AP_Mount, state._kpmin, 0.003),

    AP_GROUPINFO("_T_kPmax", 23, AP_Mount, state._kpmax, 0.006),

    AP_GROUPINFO("_T_EOaux", 24, AP_Mount, state._EOAux_ch, 5),

    AP_GROUPINFO("_T_IRaux", 25, AP_Mount, state._IRAux_ch, 6),


    AP_GROUPEND
};

AP_Mount::AP_Mount(const AP_AHRS_TYPE &ahrs, const struct Location &current_loc) :
    _ahrs(ahrs),
    _current_loc(current_loc)
{
	AP_Param::setup_object_defaults(this, var_info);

    // initialise backend pointers and mode
    _backend = nullptr;
}

// init - detect and initialise all mounts
void AP_Mount::init(const AP_SerialManager& serial_manager)
{
    // default instance's state
    state._mode = (enum MAV_MOUNT_MODE)state._default_mode.get();

    MountType mount_type = get_mount_type();

    // check for QHPayload using serial protocol    
    if (mount_type == Mount_Type_QHPayload) {
        _backend = new AP_Mount_QHPayload(*this, state);
    }

    // init new instance
    if (_backend != nullptr) {
        _backend->init(serial_manager);
        }
}

// update - give mount opportunity to update servos.  should be called at 10hz or higher
void AP_Mount::update()
{
    // update each instance
    if (_backend != nullptr) {
        _backend->update();
    }
}

// used for gimbals that need to read INS data at full rate
void AP_Mount::update_fast()
{
    // update each instance
    if (_backend != nullptr) {
        _backend->update_fast();
    }
}

// get_mount_type - returns the type of mount
AP_Mount::MountType AP_Mount::get_mount_type() const
{
    return (MountType)state._type.get();
}

// get_mode - returns current mode of mount (i.e. Retracted, Neutral, RC_Targeting, GPS Point)
MAV_MOUNT_MODE AP_Mount::get_mode() const
{
    return state._mode;
}

// set_mode_to_default - restores the mode to it's default mode held in the MNT_MODE parameter
//      this operation requires 60us on a Pixhawk/PX4
void AP_Mount::set_mode_to_default()
{
    set_mode((enum MAV_MOUNT_MODE)state._default_mode.get());
}

// set_mode - sets mount's mode
void AP_Mount::set_mode(enum MAV_MOUNT_MODE mode)
{
    // sanity check instance
    if (_backend == nullptr) {
        return;
    }

    // call backend's set_mode
    _backend->set_mode(mode);
}

// set_angle_targets - sets angle targets in degrees
void AP_Mount::set_angle_targets(float roll, float tilt, float pan)
{
    if (_backend == nullptr) {
        return;
    }

    // send command to backend
    _backend->set_angle_targets(roll, tilt, pan);
}

/// Change the configuration of the mount
/// triggered by a MavLink packet.
void AP_Mount::configure_msg(mavlink_message_t* msg)
{
    if (_backend == nullptr) {
        return;
    }

    // send message to backend
    _backend->configure_msg(msg);
}

/// Control the mount (depends on the previously set mount configuration)
/// triggered by a MavLink packet.
void AP_Mount::control_msg(mavlink_message_t *msg)
{
    if (_backend == nullptr) {
        return;
    }

    // send message to backend
    _backend->control_msg(msg);
}

void AP_Mount::control(int32_t pitch_or_lat, int32_t roll_or_lon, int32_t yaw_or_alt, enum MAV_MOUNT_MODE mount_mode)
{
    if (_backend == nullptr) {
        return;
    }

    // send message to backend
    _backend->control(pitch_or_lat, roll_or_lon, yaw_or_alt, mount_mode);
}

/// Return mount status information
void AP_Mount::status_msg(mavlink_channel_t chan)
{
    if (_backend != nullptr) {
        _backend->status_msg(chan);
    }
}

// set_roi_target - sets target location that mount should attempt to point towards
void AP_Mount::set_roi_target(const struct Location &target_loc)
{
    // call instance's set_roi_cmd
    if (_backend!= nullptr) {
        _backend->set_roi_target(target_loc);
    }
}

// pass a GIMBAL_REPORT message to the backend
void AP_Mount::handle_gimbal_report(mavlink_channel_t chan, mavlink_message_t *msg)
{
    if (_backend != nullptr) {
        _backend->handle_gimbal_report(chan, msg);
    }
}

// handle PARAM_VALUE
void AP_Mount::handle_param_value(mavlink_message_t *msg)
{
    if (_backend != nullptr) {
        _backend->handle_param_value(msg);
    }
}

// send a GIMBAL_REPORT message to the GCS
void AP_Mount::send_gimbal_report(mavlink_channel_t chan)
{
    if (_backend != nullptr) {
        _backend->send_gimbal_report(chan);
    }    
}
