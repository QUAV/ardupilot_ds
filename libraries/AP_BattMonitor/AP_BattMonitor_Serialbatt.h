#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

// DEBUG
#include <GCS_MAVLink/GCS.h>

// Definitions
#define HEADER_1BYTE 'G'
#define HEADER_2BYTE 'O'
#define HEADER_3BYTE 'V'

class AP_BattMonitor_Serialbatt : public AP_BattMonitor_Backend
{
public:

    // Constructor
    AP_BattMonitor_Serialbatt(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    // Probably the overrided method can be changed by a call to new serialmanager method. TODO
    void init(void) override {}
    void init(const AP_SerialManager& serial_manager) override;

    // Read the battery voltage and current.  Should be called at 10hz
    void read();

    // Returns true if battery monitor provides consumed energy info. Not by now.
    bool has_consumed_energy() const override { return false; }

    // Returns true if battery monitor provides current info
    bool has_current() const override { return true; }

private:

    // Parse the body of the message
    void parse_body();

    // Read incomming message from Serial battery sensor
    bool read_incoming();

    // Battery data structure
    struct PACKED Battery_Data {
        int16_t  Volts;      // cents of volt
        int16_t  amps_batt;  // tenths of ampere
        int16_t  amps_gen;   // tenths of ampere
        int16_t  amps_rot;   // tenths of ampere
        uint16_t ml_fuel;   // mililiters
        uint16_t pwm_throttle; // pwm 0-2000
    };

    // Used on parse_body. Made this way in case of future struct data aditions.
    union PACKED Data {
        DEFINE_BYTE_ARRAY_METHODS
        Battery_Data Batt_data; 
    } _buffer;

    // Pointer to the port instance designated to this driver
    AP_HAL::UARTDriver *_port;
    bool _initialised: 1;
    bool _last_command_confirmed : 1;
    uint32_t _last_reading_ms;
    
    // Variables used for protocol reading and parsing
    uint8_t _checksum;
    uint8_t _step;
    uint8_t _command_id;
    uint8_t _payload_length;
    uint8_t _payload_counter;

};
