#pragma once

#include "AP_Mount_Alexmos.h"

// Debug
#include <GCS_MAVLink/GCS.h>
// gcs().send_text(MAV_SEVERITY_CRITICAL, "RC VALUE: %5.3f", (double)zoom_visca_value);


class AP_Mount_QHPayload : public AP_Mount_Alexmos
{
public:
    //constructor
    AP_Mount_QHPayload(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance):
        AP_Mount_Alexmos(frontend, state, instance),
        _PL_port(nullptr),
        _PL_initialised(false)
    {}

    // Initialization. It also calls alexmos init
    virtual void init(const AP_SerialManager& serial_manager);

    // Update Payload related tasks. It calls alexmos update for gimbal control
    virtual void update();

    // Probably needed to add calls to alexmos public methods

private:

    // Set zoom for Sony Camera
    void PL_set_zoom();

    // Update tracker commands
    void PL_tracker_ctrl();

    // Update RC readings

    // Serial related methods
    void PL_write_params();
    void PL_send_command(uint8_t* data, uint8_t size);
    void PL_parse_body();
    void PL_read_incoming();

    struct PACKED Tracking_msg {
        uint8_t Header_1;              // 1
        uint8_t Header_2;              // 2
        uint8_t Address;               // 3
        uint8_t Byte_4;                // 4
        uint8_t Byte_5;                // 5
        uint8_t Working_state;         // 6 Image setting mode, SD mode, OSD Mode
        uint8_t ImSett_SD_OSD;         // 7 value of ^^^^
        int16_t X_Mov;                 // 8,9 
        int16_t Y_Mov;                 // 10,11 
        uint8_t Confirm_tracking;      // 12 
        uint8_t Bright_adj;            // 13 
        uint8_t Track_Square_size;     // 14 
        uint8_t Video_Source;          // 15
        uint8_t Byte_16;               // 16
        uint8_t Byte_17;               // 17
        uint8_t Byte_18;               // 18
        uint8_t Byte_19;               // 19
        uint8_t Byte_20;               // 20 pitching angle
        uint8_t Byte_21;               // 21 pitching angle
        uint8_t Byte_22;               // 22 pitching angle
        uint8_t Byte_23;               // 23 pitching angle
        uint8_t Byte_24;               // 24 viewing angle
        uint8_t Byte_25;               // 25 viewing angle
        uint8_t Byte_26;               // 26 GPS X
        uint8_t Byte_27;               // 27 GPS X
        uint8_t Byte_28;               // 28 GPS X
        uint8_t Byte_29;               // 29 GPS X
        uint8_t Byte_30;               // 30 
        uint8_t Byte_31;               // 31
        uint8_t Byte_32;               // 32
        uint8_t Byte_33;               // 33
        uint8_t Byte_34;               // 34
        uint8_t Byte_35;               // 35
        uint8_t Byte_36;               // 36
        uint8_t Byte_37;               // 37
        uint8_t Byte_38;               // 38
        uint8_t Byte_39;               // 39
        uint8_t Byte_40;               // 40
        uint8_t Byte_41;               // 41
        uint8_t Byte_42;               // 42
        uint8_t Byte_43;               // 43
        uint8_t Byte_44;               // 44
        uint8_t Byte_45;               // 45
        uint8_t Byte_46;               // 46
        uint8_t Byte_47;               // 47
        uint8_t checksum;              // 48
    } _PL_Tracking_buffer;

    // Visca message for direct zoom control
    struct PACKED FCB_zoom_msg {
        uint8_t Byte_1 = 0x81;
        uint8_t Byte_2 = 0x01;
        uint8_t Byte_3 = 0x04;
        uint8_t Byte_4 = 0x47;
        uint8_t Byte_5;       
        uint8_t Byte_6;
        uint8_t Byte_7;
        uint8_t Byte_8;
        uint8_t Byte_9 = 0xFF;
    } _PL_FCB_zoom_buffer;

    AP_HAL::UARTDriver *_PL_port;

    bool _PL_initialised : 1;

    uint16_t _Zoom_level;
};
