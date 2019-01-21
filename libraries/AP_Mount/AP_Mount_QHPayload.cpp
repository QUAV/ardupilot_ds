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

    PL_vid_src();
    PL_set_zoom();
    AP_Mount_Alexmos::update();
}


// Send zoom to Sony Camera
void AP_Mount_QHPayload::PL_set_zoom()
{
    uint16_t rc_value = RC_Channels::rc_channel(_state._Zoom_ch-1)->get_control_in();

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
    _last_zoom_rc =  RC_Channels::rc_channel(_state._Zoom_ch-1)->get_control_in();
}


// Control tracker 
void AP_Mount_QHPayload::PL_vid_src()
{
    uint16_t Video_value = RC_Channels::rc_channel(_state._Video_ch-1)->get_control_in();
    
    // check if rc has changed. If not return, no need to send to much equal commands.
    if ( Video_value < _last_vid_rc+20 && Video_value > _last_vid_rc-20 ){
        return;
    }
    
    Tracking_msg outgoing_buffer;

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
    _last_vid_rc =  RC_Channels::rc_channel(_state._Video_ch-1)->get_control_in();
}


// Write parameters to QHPayload. Here should be the video settings aside from tracker
void AP_Mount_QHPayload::PL_write_params()
{

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

}

void AP_Mount_QHPayload::PL_read_incoming()
{

}