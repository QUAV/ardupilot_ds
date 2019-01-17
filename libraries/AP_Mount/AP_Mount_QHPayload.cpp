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

    PL_tracker_ctrl();
    PL_set_zoom();
    AP_Mount_Alexmos::update();
}


// Send zoom to Sony Camera
//
// Set dead zone, only send if changed!!!
//
void AP_Mount_QHPayload::PL_set_zoom()
{
    uint16_t rc_value = RC_Channels::rc_channel(_state._Zoom_ch-1)->get_control_in();
    uint16_t zoom_visca_value = (1000-rc_value)*16.384;


    _frontend._joystick_speed = ((_state._Speed_max-_state._Speed_min)*0.001*(rc_value))+_state._Speed_min;

    int valuea = zoom_visca_value & 15;
    int valuebZ = zoom_visca_value >> 4;
    int valuecZ = zoom_visca_value >> 8;
    int valuedZ = zoom_visca_value >> 12;
    int valueb = valuebZ & 15  ;
    int valuec = valuecZ & 15 ;
    int valued = valuedZ & 15 ;

    _PL_FCB_zoom_buffer.Byte_5 =  valued;
    _PL_FCB_zoom_buffer.Byte_6 =  valuec;
    _PL_FCB_zoom_buffer.Byte_7 =  valueb;
    _PL_FCB_zoom_buffer.Byte_8 =  valuea;

    PL_send_command((uint8_t *)&_PL_FCB_zoom_buffer, sizeof(_PL_FCB_zoom_buffer));
}


// Control tracker 
void AP_Mount_QHPayload::PL_tracker_ctrl()
{
    uint16_t Video_value = RC_Channels::rc_channel(_state._Video_ch-1)->get_control_in();

    if ( Video_value > 750 ){
        _PL_Tracking_buffer.Video_Source = 0x00;
    } else if ( Video_value > 500 ){
        _PL_Tracking_buffer.Video_Source = 0x01;
    } else if ( Video_value > 250 ){
        _PL_Tracking_buffer.Video_Source = 0x02;
    } else {
        _PL_Tracking_buffer.Video_Source = 0x03;
    }

    _PL_Tracking_buffer.Header_1         = 0x7E;     // 1
    _PL_Tracking_buffer.Header_2         = 0x7E;     // 2
    _PL_Tracking_buffer.Address          = 0x44;     // 3
    _PL_Tracking_buffer.Byte_4           = 0x00;     // 4
    _PL_Tracking_buffer.Byte_5           = 0x00;     // 5
    _PL_Tracking_buffer.Working_state    = 0x78;     // 6 Image setting mode, SD mode, OSD Mode
    _PL_Tracking_buffer.ImSett_SD_OSD    = 0x00;     // 7 value of ^^^^
    _PL_Tracking_buffer.X_Mov            = 0x00;     // 8,9 
    _PL_Tracking_buffer.Y_Mov            = 0x00;     // 10,11 
    _PL_Tracking_buffer.Confirm_tracking = 0x00;     // 12 
    _PL_Tracking_buffer.Bright_adj       = 0x00;     // 13 
    _PL_Tracking_buffer.Track_Square_size= 0x00;     // 14 
    //_PL_Tracking_buffer.Video_Source     = 0x00;     // 15
    _PL_Tracking_buffer.Byte_16          = 0x00;     // 16
    _PL_Tracking_buffer.Byte_17          = 0x00;     // 17
    _PL_Tracking_buffer.Byte_18          = 0x00;     // 18
    _PL_Tracking_buffer.Byte_19          = 0x00;     // 19
    _PL_Tracking_buffer.Byte_20          = 0x00;     // 20 pitching angle
    _PL_Tracking_buffer.Byte_21          = 0x00;     // 21 pitching angle
    _PL_Tracking_buffer.Byte_22          = 0x00;     // 22 pitching angle
    _PL_Tracking_buffer.Byte_23          = 0x00;     // 23 pitching angle
    _PL_Tracking_buffer.Byte_24          = 0x00;     // 24 viewing angle
    _PL_Tracking_buffer.Byte_25          = 0x00;     // 25 viewing angle
    _PL_Tracking_buffer.Byte_26          = 0x00;     // 26 GPS X
    _PL_Tracking_buffer.Byte_27          = 0x00;     // 27 GPS X
    _PL_Tracking_buffer.Byte_28          = 0x00;     // 28 GPS X
    _PL_Tracking_buffer.Byte_29          = 0x00;     // 29 GPS X
    _PL_Tracking_buffer.Byte_30          = 0x00;     // 30 
    _PL_Tracking_buffer.Byte_31          = 0x00;     // 31
    _PL_Tracking_buffer.Byte_32          = 0x00;     // 32
    _PL_Tracking_buffer.Byte_33          = 0x00;     // 33
    _PL_Tracking_buffer.Byte_34          = 0x00;     // 34
    _PL_Tracking_buffer.Byte_35          = 0x00;     // 35
    _PL_Tracking_buffer.Byte_36          = 0x00;     // 36
    _PL_Tracking_buffer.Byte_37          = 0x00;     // 37
    _PL_Tracking_buffer.Byte_38          = 0x00;     // 38
    _PL_Tracking_buffer.Byte_39          = 0x00;     // 39
    _PL_Tracking_buffer.Byte_40          = 0x00;     // 40
    _PL_Tracking_buffer.Byte_41          = 0x00;     // 41
    _PL_Tracking_buffer.Byte_42          = 0x00;     // 42
    _PL_Tracking_buffer.Byte_43          = 0x00;     // 43
    _PL_Tracking_buffer.Byte_44          = 0x00;     // 44
    _PL_Tracking_buffer.Byte_45          = 0x00;     // 45
    _PL_Tracking_buffer.Byte_46          = 0x00;     // 46
    _PL_Tracking_buffer.Byte_47          = 0x00;     // 47

    _PL_Tracking_buffer.checksum = _PL_Tracking_buffer.Header_1+_PL_Tracking_buffer.Header_2+_PL_Tracking_buffer.Address+_PL_Tracking_buffer.Working_state+_PL_Tracking_buffer.Video_Source;

    PL_send_command((uint8_t *)&_PL_Tracking_buffer, sizeof(_PL_Tracking_buffer));

}


// Write parameters to QHPayload. Here should be the video settings aside from tracker
void AP_Mount_QHPayload::PL_write_params()
{

}

// Send command to QHPayload API
//
// Check carefully size!! size + header, address, etc
//
//
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