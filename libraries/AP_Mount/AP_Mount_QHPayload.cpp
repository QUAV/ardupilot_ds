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
    _last_zoom_rc = RC_Channels::rc_channel(_state._Zoom_ch-1)->get_control_in();
    _last_vid_rc = RC_Channels::rc_channel(_state._Video_ch-1)->get_control_in();
    _last_rec_rc = RC_Channels::rc_channel(_state._Rec_ch-1)->get_control_in();
    _rec_state = Standby;
    _vid_mode = EOplusIR;
    _track_state = NoTracking;
}

// Update method. It is called periodically, at 60 Hz
void AP_Mount_QHPayload::update()
{
//    uint32_t timeprov;
//    uint32_t timestart = AP_HAL::micros();
//    timeprov = AP_HAL::micros()-timestart;
//    gcs().send_text(MAV_SEVERITY_CRITICAL, "init: %5.3f", (dou//ble)timeprov);
//    timestart = AP_HAL::micros();

    if (!_PL_initialised) {
        return;
    }
    PL_read_incoming();
    PL_check_rc();        
    PL_update_angle_targets();
    AP_Mount_Alexmos::update();

}

// check rc
void AP_Mount_QHPayload::PL_check_rc()
{
    // Rec functionWaf: Leaving directory `/home/sas/ardupilot2/build/fmuv5'

    uint16_t rc_val = RC_Channels::rc_channel(_state._Rec_ch-1)->get_control_in();

    if ( rc_val > _last_rec_rc+20 || rc_val < _last_rec_rc-20 ) {

        _last_rec_rc = rc_val;

        if ( rc_val > 800 && _rec_state != Recording ){
            _rec_state = Recording;
        } else if ( rc_val < 700 && rc_val > 300 && _rec_state != Standby ) {
            _rec_state = Standby;
        } else if ( rc_val < 200 && _rec_state != Snapshot) {
            _rec_state = Snapshot;
        }

        PL_rec();
    }

    // Set zoom function
    rc_val = RC_Channels::rc_channel(_state._Zoom_ch-1)->get_control_in();

    if ( rc_val > _last_zoom_rc+5 || rc_val < _last_zoom_rc-5 ){

        _last_zoom_rc = rc_val;
        // adjust gain for tracker according zoom
        _kp = ((_state._kpmax-_state._kpmin)*0.001*(rc_val))+_state._kpmin;
        
        // adjust velocity for manual pointing according zoom
        _frontend._joystick_speed = ((_state._Speed_max-_state._Speed_min)*0.001*(rc_val))+_state._Speed_min;
        
        _EOzoom = (1000-rc_val)*16.384;

        gcs().send_text(MAV_SEVERITY_INFO, "zoom: %5.3f", (double)_EOzoom);

        PL_set_EOzoom();
    }

    // Set video mode
    rc_val = RC_Channels::rc_channel(_state._Video_ch-1)->get_control_in();

    if ( rc_val > _last_vid_rc+20 || rc_val < _last_vid_rc-20 ){

        _last_vid_rc = rc_val;

        if ( _last_vid_rc > 900 ){
            _vid_mode = EO;
        } else if ( _last_vid_rc > 500 ) {
            _vid_mode = EOplusIR;
        } else if ( _last_vid_rc > 100 ) {
            _vid_mode = IR;
        } else {
            _vid_mode = IRplusEO;
        }

        PL_vid_src();
    }

    rc_val = RC_Channels::rc_channel(_state._Track_ch-1)->get_control_in();

    if ( rc_val > 700 && _track_high == 0 ) {
        _track_high = 1;

        PL_tracker();

    } else if ( rc_val < 700 && _track_high == 1 ) {
        _track_high = 0;
    }
}

// Start/Stop recording
void AP_Mount_QHPayload::PL_rec()
{
    Type2_msg outgoing_buffer;

    switch(_rec_state){

        case Standby:
            outgoing_buffer.ImSett_SD_OSD = 0x00;
            _rec_state = Standby;
            break;

        case Recording:
            outgoing_buffer.ImSett_SD_OSD = 0x01;
            _rec_state = Recording;
            break;

        case Snapshot:
            outgoing_buffer.ImSett_SD_OSD = 0x02;
            _rec_state = Snapshot;
            break;
    }

    outgoing_buffer.Working_state = 0x7C;
    outgoing_buffer.checksum =  outgoing_buffer.Header_1+
                                outgoing_buffer.Header_2+
                                outgoing_buffer.Address+
                                outgoing_buffer.Working_state+
                                outgoing_buffer.ImSett_SD_OSD;

    PL_send_command((uint8_t *)&outgoing_buffer, sizeof(outgoing_buffer));
}

// Send zoom to Sony Camera
void AP_Mount_QHPayload::PL_set_EOzoom()
{
    // this is stuff for the visca protocol for zooming
    int val1  = _EOzoom &  15;
    int val22 = _EOzoom >> 4;
    int val33 = _EOzoom >> 8;
    int val44 = _EOzoom >> 12;
    int val2  = val22   &  15;
    int val3  = val33   &  15;
    int val4  = val44   &  15;

    // prepare buffer
    EO_zoom_msg outgoing_buffer;
    outgoing_buffer.Byte_5 = val4;
    outgoing_buffer.Byte_6 = val3;
    outgoing_buffer.Byte_7 = val2;
    outgoing_buffer.Byte_8 = val1;

    PL_send_command((uint8_t *)&outgoing_buffer, sizeof(outgoing_buffer));
}

// Control vid source
void AP_Mount_QHPayload::PL_vid_src()
{
    Type1_msg outgoing_buffer;

    switch(_vid_mode){

        case EO:
            outgoing_buffer.Video_Source = 0x00;
            break;

        case IRplusEO:
            outgoing_buffer.Video_Source = 0x01;
            break;

        case IR:
            outgoing_buffer.Video_Source = 0x02;
            break;

        case EOplusIR:
            outgoing_buffer.Video_Source = 0x03;
            break;
    }

    outgoing_buffer.Working_state = 0x78; 
    outgoing_buffer.checksum = outgoing_buffer.Header_1+
                               outgoing_buffer.Header_2+
                               outgoing_buffer.Address+
                               outgoing_buffer.Working_state+
                               outgoing_buffer.Video_Source;  // on constructor already implemented headers and address

    PL_send_command((uint8_t *)&outgoing_buffer, sizeof(outgoing_buffer));
}


// Update tracker
void AP_Mount_QHPayload::PL_tracker()
{
    Type1_msg outgoing_buffer;

    switch (_track_state) {

        case NoTracking:
            outgoing_buffer.Working_state = 0x71;
            outgoing_buffer.Confirm_tracking = 0x01;
            _track_state = Tracking;
            break;

        case Tracking:
            outgoing_buffer.Working_state = 0x26;
            outgoing_buffer.Confirm_tracking = 0x00;
            _track_state = NoTracking;
            break;
    }

    outgoing_buffer.checksum = outgoing_buffer.Header_1+
                               outgoing_buffer.Header_2+
                               outgoing_buffer.Address+
                               outgoing_buffer.Working_state+
                               outgoing_buffer.Confirm_tracking;

    PL_send_command((uint8_t *)&outgoing_buffer, sizeof(outgoing_buffer));
}


// Update angle targets for Alexmos to reach
void AP_Mount_QHPayload::PL_update_angle_targets()
{
    // if joystick_speed is defined then pilot input defines a rate of change of the angle
    if ( _track_msg_rdy ) {
        if ( _track_state == Tracking ){
            float tilt = PL_tracker_PID(_Track_Y);
            float pan = PL_tracker_PID(_Track_X);

            _track_msg_rdy = false;

            _angle_ef_target_rad.y += tilt * 0.0001f * _frontend._joystick_speed;
            _angle_ef_target_rad.y = constrain_float(_angle_ef_target_rad.y, radians(_state._tilt_angle_min*0.01f), radians(_state._tilt_angle_max*0.01f));
            
            _angle_ef_target_rad.z += pan * 0.0001f * _frontend._joystick_speed;
        }
    }
}

// Tracker PIDS
float AP_Mount_QHPayload::PL_tracker_PID(float error)
{
    // Compute proportional component
    _PID_info.P = error * _kp;
    float output = _PID_info.P;

    return output;
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

    _track_msg_rdy = true;
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