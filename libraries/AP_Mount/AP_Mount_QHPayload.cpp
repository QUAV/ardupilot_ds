#include "AP_Mount_QHPayload.h"

extern const AP_HAL::HAL& hal;

// Initialization
void AP_Mount_QHPayload::init(const AP_SerialManager& serial_manager)
{
    // check for QHPayload protocol
    if ((_PL_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_QHPayload, 0))) {
        _PL_initialised = true;
    }

    AP_Mount_Alexmos::init(serial_manager);
}

// Update method. It is called periodically, at 60 Hz
void AP_Mount_QHPayload::update()
{
    if (!_PL_initialised) {
        return;
    }

    PL_read_incoming();
    PL_vid_src();
    PL_set_zoom();
    PL_rec();
    PL_tracker();
    PL_update_angle_targets();
    
    AP_Mount_Alexmos::update();
}

// Start/Stop recording
void AP_Mount_QHPayload::PL_rec()
{
    uint16_t rc_value = RC_Channels::rc_channel(_state._Rec_ch-1)->get_control_in();

    if ( rc_value < _last_rec_rc+20 && rc_value > _last_rec_rc-20 ){
        return;
    }

    _last_rec_rc = rc_value;

    if ( rc_value > 800 && _rec_state == Recording ){
        Type2_msg outgoing_buffer;
        outgoing_buffer.Working_state = 0x7C;
        outgoing_buffer.ImSett_SD_OSD = 0x00;
        outgoing_buffer.checksum =  outgoing_buffer.Header_1+outgoing_buffer.Header_2+outgoing_buffer.Address+outgoing_buffer.Working_state+outgoing_buffer.ImSett_SD_OSD;
        PL_send_command((uint8_t *)&outgoing_buffer, sizeof(outgoing_buffer));
        _rec_state = Standby;

    } else if ( rc_value < 200 && _rec_state == Standby ) {
        Type2_msg outgoing_buffer;
        outgoing_buffer.Working_state = 0x7C;
        outgoing_buffer.ImSett_SD_OSD = 0x01;
        outgoing_buffer.checksum =  outgoing_buffer.Header_1+outgoing_buffer.Header_2+outgoing_buffer.Address+outgoing_buffer.Working_state+outgoing_buffer.ImSett_SD_OSD+outgoing_buffer.Byte_8+outgoing_buffer.Byte_9;
        PL_send_command((uint8_t *)&outgoing_buffer, sizeof(outgoing_buffer));
        _rec_state = Recording;

    } else {
        return;
    }
}

// Send zoom to Sony Camera
void AP_Mount_QHPayload::PL_set_zoom()
{
    uint16_t rc_value = RC_Channels::rc_channel(_state._Zoom_ch-1)->get_control_in();

    _scaler = (1000-rc_value)*0.0001f*_state._scl;

    // check if rc has changed. If not return, no need to send to much equal commands.
    if ( rc_value < _last_zoom_rc+5 && rc_value > _last_zoom_rc-5 ){
        return;
    }

    // Set speed sensibility
    _frontend._joystick_speed = ((_state._Speed_max-_state._Speed_min)*0.001*(rc_value))+_state._Speed_min;

    // Visca needs 0x4000 as maximum zoom cmd for 30x zoom. More than that is digital zoom
    uint16_t zoom_visca_value = (1000-rc_value)*16.384;

    FCB_zoom_msg outgoing_buffer;

    int valuea = zoom_visca_value & 15;
    int valuebZ = zoom_visca_value >> 4;
    int valuecZ = zoom_visca_value >> 8;
    int valuedZ = zoom_visca_value >> 12;
    int valueb = valuebZ & 15  ;
    int valuec = valuecZ & 15 ;
    int valued = valuedZ & 15 ;

    outgoing_buffer.Byte_5 =  valued;
    outgoing_buffer.Byte_6 =  valuec;
    outgoing_buffer.Byte_7 =  valueb;
    outgoing_buffer.Byte_8 =  valuea;

    PL_send_command((uint8_t *)&outgoing_buffer, sizeof(outgoing_buffer));
    _last_zoom_rc =  rc_value;
}


// Control tracker 
void AP_Mount_QHPayload::PL_vid_src()
{
    uint16_t Video_value = RC_Channels::rc_channel(_state._Video_ch-1)->get_control_in();
    
    // check if rc has changed. If not return, no need to send to much equal commands.
    if ( Video_value < _last_vid_rc+20 && Video_value > _last_vid_rc-20 ){
        return;
    }
    
    Type1_msg outgoing_buffer;

    if ( Video_value > 750 ){
        outgoing_buffer.Video_Source = 0x00;
    } else if ( Video_value > 500 ){
        outgoing_buffer.Video_Source = 0x01;
    } else if ( Video_value > 250 ){
        outgoing_buffer.Video_Source = 0x02;
    } else {
        outgoing_buffer.Video_Source = 0x03;
    }

    outgoing_buffer.Working_state    = 0x78;                                                   // 6 Image setting mode, SD mode, OSD Mode
    outgoing_buffer.checksum = outgoing_buffer.Header_1+outgoing_buffer.Header_2+outgoing_buffer.Address+outgoing_buffer.Working_state+outgoing_buffer.Video_Source;  // on constructor already implemented headers and address

    PL_send_command((uint8_t *)&outgoing_buffer, sizeof(outgoing_buffer));
    _last_vid_rc =  Video_value;
}


// Update tracker
void AP_Mount_QHPayload::PL_tracker()
{
    uint16_t rc = RC_Channels::rc_channel(_state._Track_ch-1)->get_control_in();

    if ( rc > 700 && _track_high == 0 ) {

        _track_high = 1;
        Type1_msg outgoing_buffer;

        switch (_track_state) {

            case NoTracking:

                outgoing_buffer.Working_state = 0x71;
                outgoing_buffer.Confirm_tracking = 0x01;
                outgoing_buffer.checksum = outgoing_buffer.Header_1+outgoing_buffer.Header_2+outgoing_buffer.Address+outgoing_buffer.Working_state+outgoing_buffer.Confirm_tracking;
                PL_send_command((uint8_t *)&outgoing_buffer, sizeof(outgoing_buffer));

                _track_state = Tracking;
                break;

            case Tracking:

                outgoing_buffer.Working_state = 0x71;
                outgoing_buffer.Confirm_tracking = 0x00;
                outgoing_buffer.checksum = outgoing_buffer.Header_1+outgoing_buffer.Header_2+outgoing_buffer.Address+outgoing_buffer.Working_state+outgoing_buffer.Confirm_tracking;
                PL_send_command((uint8_t *)&outgoing_buffer, sizeof(outgoing_buffer));

                _track_state = NoTracking;
                break;
        }
    } else if ( rc < 700 && _track_high == 1 ) {
        _track_high = 0;
    }
}


// Update angle targets for Alexmos to reach
void AP_Mount_QHPayload::PL_update_angle_targets()
{
    // if joystick_speed is defined then pilot input defines a rate of change of the angle
    if (_frontend._joystick_speed) {

        if ( _track_state == Tracking ){
            float tilt = PL_tracker_PID(_Track_Y);
            float pan = PL_tracker_PID(_Track_X);

                _angle_ef_target_rad.y += tilt * 0.0001f * _frontend._joystick_speed;
                _angle_ef_target_rad.y = constrain_float(_angle_ef_target_rad.y, radians(_state._tilt_angle_min*0.01f), radians(_state._tilt_angle_max*0.01f));

                _angle_ef_target_rad.z += pan * 0.0001f * _frontend._joystick_speed;
                _angle_ef_target_rad.z = constrain_float(_angle_ef_target_rad.z, radians(_state._pan_angle_min*0.01f), radians(_state._pan_angle_max*0.01f));
        }
    }
}


// Tracker PIDS
float AP_Mount_QHPayload::PL_tracker_PID(float error)
{
    uint32_t tnow = AP_HAL::millis();
    uint32_t dt = tnow - _last_t;
    float output = 0;
    float delta_time;

    if (_last_t == 0 || dt > 1000) {
        dt = 0;

		// if this PID hasn't been used for a full second then zero
		// the intergator term. This prevents I buildup from a
		// previous fight mode from causing a massive return before
		// the integrator gets a chance to correct itself
		PL_reset_I();
    }
    _last_t = tnow;

    float scaler = _state._scl;

    delta_time = (float)dt / 1000.0f;

    // Compute proportional component
    _PID_info.P = error * _state._kp;
    output += _PID_info.P;

    // Compute derivative component if time has elapsed
    if ((fabsf(_state._kd) > 0) && (dt > 0)) {
        float derivative;

		if (isnan(_last_derivative)) {
			// we've just done a reset, suppress the first derivative
			// term as we don't want a sudden change in input to cause
			// a large D output change			
			derivative = 0;
			_last_derivative = 0;
		} else {
			derivative = (error - _last_error) / delta_time;
		}

        // discrete low pass filter, cuts out the
        // high frequency noise that can drive the controller crazy
        float RC = 1/(2*M_PI*_state._fCut);
        derivative = _last_derivative +
                     ((delta_time / (RC + delta_time)) *
                      (derivative - _last_derivative));

        // update state
        _last_error             = error;
        _last_derivative    = derivative;

        // add in derivative component
        _PID_info.D = _state._kd * derivative;
        output                          += _PID_info.D;
    }

    // scale the P and D components
    output *= scaler;
    _PID_info.D *= scaler;
    _PID_info.P *= scaler;

    // Compute integral component if time has elapsed
    if ((fabsf(_state._ki) > 0) && (dt > 0)) {
        _integrator             += (error * _state._ki) * scaler * delta_time;
        if (_integrator < -_state._imax) {
            _integrator = -_state._imax;
        } else if (_integrator > _state._imax) {
            _integrator = _state._imax;
        }
        _PID_info.I = _integrator;
        output += _integrator;
    }

    return output;
}


// reset tracker I therm
void AP_Mount_QHPayload::PL_reset_I()
{
    _integrator = 0;
	// we use NAN (Not A Number) to indicate that the last 
	// derivative value is not valid
    _last_derivative = NAN;
    _PID_info.I = 0;
}

// Send command to QHPayload API
void AP_Mount_QHPayload::PL_send_command(uint8_t* data, uint8_t size)
{
    if (_PL_port->txspace() < (size)) {
        return;
    }
    for (uint8_t i = 0;  i != size ; i++) {
        _PL_port->write( data[i] );
    }
}

void AP_Mount_QHPayload::PL_parse_body()
{
    _Track_X = _PL_buffer.msg.X;
    _Track_Y = _PL_buffer.msg.Y;

    // PARSE STATUS?
}

void AP_Mount_QHPayload::PL_read_incoming()
{
    uint8_t data;
    int16_t numc;

    numc = _PL_port->available();

    if (numc < 0) {
        return;
    }

    for (int16_t i = 0; i < numc; i++) {
        
        data = _PL_port->read();
        switch (_PL_step) {
            case 0:
                if ( 0xBB == data ) {
                    _PL_step = 1;
                    _PL_checksum = data;
                }
                break;
            
            case 1:
                if ( 0x01 == data ) {
                    _PL_step++;
                    _PL_checksum += data;
                }
                break;

            case 2:
                _PL_checksum += data;
                if (_PL_counter < sizeof(_PL_buffer)) {
                    _PL_buffer[_PL_counter] = data;
                }
                if (++_PL_counter == 5) {
                    _PL_step++;
                }
                break;
            
            case 3:
                _PL_step = 0;
                _PL_counter = 0;
                if (_PL_checksum != data) {                  
                    _PL_checksum = 0;
                    break;
                }
                _PL_checksum = 0;
                PL_parse_body();
        }
    }
}


// Write parameters to QHPayload. Here should be the video settings aside from tracker
void AP_Mount_QHPayload::PL_write_params()
{

}