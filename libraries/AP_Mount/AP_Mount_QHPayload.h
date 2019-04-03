#pragma once

#include "AP_Mount_Alexmos.h"

// ---------------------------------Debug----------------------------------------------
#include <GCS_MAVLink/GCS.h>
// gcs().send_text(MAV_SEVERITY_CRITICAL, "RC VALUE: %5.3f", (double)zoom_visca_value);
//--------------------------------------------------------------------------------------


class AP_Mount_QHPayload : public AP_Mount_Alexmos
{
public:
    //constructor
    AP_Mount_QHPayload(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance):
        AP_Mount_Alexmos(frontend, state, instance),
        _PL_port(nullptr),
        _PL_initialised(false),
        _track_msg_rdy(false),
        _rec_state(Standby),
        _track_state(NoTracking),
        _last_rec_rc(0),
        _last_vid_rc(0),
        _last_zoom_rc(0),
        _track_high(0),
        _PL_step(0),
        _PL_counter(0),
        _Track_X(0),
        _Track_Y(0)
    {}

    // Initialization. It also calls alexmos init
    virtual void init(const AP_SerialManager& serial_manager);

    // Update Payload related tasks. It calls alexmos update for gimbal control
    virtual void update();

    // Probably needed to add calls to alexmos public methods

private:

    enum RecordingState {
        Standby = 0,
        Recording = 1,
        Snapshot = 2
    };

    enum TrackingState {
        NoTracking = 0,
        Tracking = 1
    };

    enum VideoMode {
        EO = 0,
        IRplusEO = 1,
        IR = 2,
        EOplusIR = 3
    };

    // Read rc functions
    void PL_check_rc();

    // Start/Stop recording
    void PL_rec();

    // Set zoom for Sony Camera
    void PL_set_EOzoom();

    // Update video source 
    void PL_vid_src();

    // Update tracker commands
    void PL_tracker();

    // Update angle targets for tracker
    void PL_update_angle_targets();

    // Tracker PID
    float PL_tracker_PID(float error);

    // Serial related methods
    void PL_write_params();
    void PL_send_command(uint8_t* data, uint8_t size);
    void PL_parse_body();
    void PL_read_incoming();

    struct PID_Info {
        float desired;
        float P;
        float I;
        float D;
        float FF;
        float AFF;
    } _PID_info;

    struct PACKED Type1_msg {
        uint8_t Header_1           = 0x7E;               // 1
        uint8_t Header_2           = 0x7E;               // 2
        uint8_t Address            = 0x44;               // 3
        uint8_t Byte_4             = 0x00;               // 4
        uint8_t Byte_5             = 0x00;               // 5
        uint8_t Working_state      = 0x00;               // 6 Image setting mode, SD mode, OSD Mode
        uint8_t ImSett_SD_OSD      = 0x00;               // 7 value of ^^^^
        int16_t X_Mov              = 0x00;               // 8,9 
        int16_t Y_Mov              = 0x00;               // 10,11 
        uint8_t Confirm_tracking   = 0x00;               // 12 
        uint8_t Bright_adj         = 0x00;               // 13 
        uint8_t Track_Square_size  = 0x00;               // 14 
        uint8_t Video_Source       = 0x00;               // 15
        uint8_t Byte_16            = 0x00;               // 16
        uint8_t Byte_17             = 0x00;               // 17
        uint8_t Byte_18            = 0x00;               // 18
        uint8_t Byte_19            = 0x00;               // 19
        uint8_t Byte_20            = 0x00;               // 20 pitching angle
        uint8_t Byte_21            = 0x00;               // 21 pitching angle
        uint8_t Byte_22            = 0x00;               // 22 pitching angle
        uint8_t Byte_23            = 0x00;               // 23 pitching angle
        uint8_t Byte_24            = 0x00;               // 24 viewing angle
        uint8_t Byte_25            = 0x00;               // 25 viewing angle
        uint8_t Byte_26            = 0x00;               // 26 GPS X
        uint8_t Byte_27            = 0x00;               // 27 GPS X
        uint8_t Byte_28            = 0x00;               // 28 GPS X
        uint8_t Byte_29            = 0x00;               // 29 GPS X
        uint8_t Byte_30            = 0x00;               // 30 
        uint8_t Byte_31            = 0x00;               // 31
        uint8_t Byte_32            = 0x00;               // 32
        uint8_t Byte_33            = 0x00;               // 33
        uint8_t Byte_34            = 0x00;               // 34
        uint8_t Byte_35            = 0x00;               // 35
        uint8_t Byte_36            = 0x00;               // 36
        uint8_t Byte_37            = 0x00;               // 37
        uint8_t Byte_38            = 0x00;               // 38
        uint8_t Byte_39            = 0x00;               // 39
        uint8_t Byte_40            = 0x00;               // 40
        uint8_t Byte_41            = 0x00;               // 41
        uint8_t Byte_42            = 0x00;               // 42
        uint8_t Byte_43            = 0x00;               // 43
        uint8_t Byte_44            = 0x00;               // 44
        uint8_t Byte_45            = 0x00;               // 45
        uint8_t Byte_46            = 0x00;               // 46
        uint8_t Byte_47            = 0x00;               // 47
        uint8_t checksum           = 0X00;                 // 48 initialize with header and address
    };

    struct PACKED Type2_msg {
        uint8_t Header_1           = 0x7E;               // 1
        uint8_t Header_2           = 0x7E;               // 2
        uint8_t Address            = 0x44;               // 3
        uint8_t Byte_4             = 0x00;               // 4
        uint8_t Byte_5             = 0x00;               // 5
        uint8_t Working_state      = 0x00;               // 6 Image setting mode, SD mode, OSD Mode
        uint8_t ImSett_SD_OSD      = 0x00;               // 7 value of ^^^^
        uint8_t Byte_8             = 0x00;               // 8   
        uint8_t Byte_9             = 0x00;               // 9   
        uint8_t Byte_10            = 0x00;               // 10   
        uint8_t Byte_11            = 0x00;               // 11   
        uint8_t Confirm_tracking   = 0x00;               // 12 
        uint8_t Bright_adj         = 0x00;               // 13 
        uint8_t Track_Square_size  = 0x00;               // 14 
        uint8_t Video_Source       = 0x00;               // 15
        uint8_t Byte_16            = 0x00;               // 16
        uint8_t Byte_17            = 0x00;               // 17
        uint8_t Byte_18            = 0x00;               // 18
        uint8_t Byte_19            = 0x00;               // 19
        uint8_t Byte_20            = 0x00;               // 20 pitching angle
        uint8_t Byte_21            = 0x00;               // 21 pitching angle
        uint8_t Byte_22            = 0x00;               // 22 pitching angle
        uint8_t Byte_23            = 0x00;               // 23 pitching angle
        uint8_t Byte_24            = 0x00;               // 24 viewing angle
        uint8_t Byte_25            = 0x00;               // 25 viewing angle
        uint8_t Byte_26            = 0x00;               // 26 GPS X
        uint8_t Byte_27            = 0x00;               // 27 GPS X
        uint8_t Byte_28            = 0x00;               // 28 GPS X
        uint8_t Byte_29            = 0x00;               // 29 GPS X
        uint8_t Byte_30            = 0x00;               // 30 
        uint8_t Byte_31            = 0x00;               // 31
        uint8_t Byte_32            = 0x00;               // 32
        uint8_t Byte_33            = 0x00;               // 33
        uint8_t Byte_34            = 0x00;               // 34
        uint8_t Byte_35            = 0x00;               // 35
        uint8_t Byte_36            = 0x00;               // 36
        uint8_t Byte_37            = 0x00;               // 37
        uint8_t Byte_38            = 0x00;               // 38
        uint8_t Byte_39            = 0x00;               // 39
        uint8_t Byte_40            = 0x00;               // 40
        uint8_t Byte_41            = 0x00;               // 41
        uint8_t Byte_42            = 0x00;               // 42
        uint8_t Byte_43            = 0x00;               // 43
        uint8_t Byte_44            = 0x00;               // 44
        uint8_t Byte_45            = 0x00;               // 45
        uint8_t Byte_46            = 0x00;               // 46
        uint8_t Byte_47            = 0x00;               // 47
        uint8_t checksum           = 0X00;                 // 48 initialize with header and address
    };

    // Visca message for direct zoom control
    struct PACKED EO_zoom_msg {
        uint8_t Byte_1 = 0x81;
        uint8_t Byte_2 = 0x01;
        uint8_t Byte_3 = 0x04;
        uint8_t Byte_4 = 0x47;
        uint8_t Byte_5;       
        uint8_t Byte_6;
        uint8_t Byte_7;
        uint8_t Byte_8;
        uint8_t Byte_9 = 0xFF;
    };

    struct PACKED tracker_feedback_msg {
        int16_t X;
        int16_t Y;
        uint8_t  Status;
    };

    union PACKED  tracker_union {
        DEFINE_BYTE_ARRAY_METHODS
        tracker_feedback_msg msg;
    } _PL_buffer;

    AP_HAL::UARTDriver *_PL_port;

    bool _PL_initialised : 1;
    bool _track_high : 1;
    bool _track_msg_rdy : 1;

    uint8_t _PL_step;
    uint8_t _PL_checksum;
    uint8_t _PL_counter;

    RecordingState _rec_state;
    TrackingState _track_state;
    VideoMode _vid_mode;

    uint16_t _last_zoom_rc;
    uint16_t _last_vid_rc;
    uint16_t _last_rec_rc;

    int16_t _Track_X;
    int16_t _Track_Y;

    float _kp;

    uint16_t _EOzoom;

    uint32_t _last_t;

};
