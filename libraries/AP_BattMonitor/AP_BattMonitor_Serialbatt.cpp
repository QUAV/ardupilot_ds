#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Serialbatt.h"

extern const AP_HAL::HAL& hal;


// Constructor

AP_BattMonitor_Serialbatt::AP_BattMonitor_Serialbatt(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params)
{
    _initialised = false;
    
    // always healthy
    _state.healthy = true; 
}


// Initialization

void AP_BattMonitor_Serialbatt::init(const AP_SerialManager& serial_manager)
{
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Serialbatt, 0))) {        
        _initialised = true;
        _step = 0;
        _last_reading_ms = 0;
    }
}


// Read info from sensor. This is the method called by front end at 10 Hz

void AP_BattMonitor_Serialbatt::read()
{
    if (!_initialised) {
        return;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "numc %5.3f", (double)_port->available());

    // Request next message
    _port->write((const uint8_t*)"+>", 2);

    if (read_incoming()){
        
        _state.healthy = true;

        gcs().send_text(MAV_SEVERITY_INFO, "Volts: %5.3f Amps_batt: %5.3f", (double)_state.voltage, (double)_state.current_amps);

        gcs().send_text(MAV_SEVERITY_INFO, "Amps_gen: %5.3f Amps_rot: %5.3f", (double)_state.generator_amps, (double)_state.rotor_amps);
        
        uint32_t tnow = AP_HAL::micros();
        float dt = tnow - _state.last_time_micros;

        // consumed mah and watts
        if (_state.last_time_micros != 0 && dt < 2000000.0f) {
                // .0002778 is 1/3600 (conversion to hours)
                float mah = _state.current_amps * dt * 0.0000002778f;
                _state.consumed_mah += mah;
                _state.consumed_wh  += 0.001f * mah * _state.voltage;
            }

        // record time
        _state.last_time_micros = tnow;
        _last_reading_ms = AP_HAL::millis();
        return;
    }

    if (AP_HAL::millis() - _last_reading_ms > 2000) {

        gcs().send_text(MAV_SEVERITY_INFO, "TIME SINCE LAST READING %5.3f", (double)(AP_HAL::millis() - _last_reading_ms));
        // need to idle this
        gcs().send_text(MAV_SEVERITY_CRITICAL, "no governor readings for more than 2 sec");
        _state.healthy = false;
        _state.voltage = 0;
        _state.current_amps = 0;
        _state.generator_amps = 0;
        _state.rotor_amps = 0;
    }
}


// Parse body of the message received. In case of future aditions, implement _comand_id, as AP_Mount libraries.

void AP_BattMonitor_Serialbatt::parse_body()
{
    // TODO!! ofset, scalar, posible power consumption, etc
    // TODO!! to cope with several current readings, probably modify AP_Battmonitor.cpp methods calling backend getters.
    _state.voltage         = 0.01f * (float)_buffer.Batt_data.Volts;
    _state.current_amps   = 0.1f * (float)_buffer.Batt_data.amps_batt;
    _state.generator_amps = 0.1f * (float)_buffer.Batt_data.amps_gen;
    _state.rotor_amps = 0.1f * (float)_buffer.Batt_data.amps_rot;
    _state.fuel_level = (float)_buffer.Batt_data.ml_fuel;
    //_state. = _buffer.Batt_data.throttle;
}


// Read inconmming message

bool AP_BattMonitor_Serialbatt::read_incoming()
{
    uint8_t data;
    int16_t numc;

    numc = _port->available();

    if (numc < 0){
        return false;
    }

    // int16_t in case of a longer data structure in the future
    for (int16_t i = 0; i < numc; i++){
        data = _port->read();
        switch (_step) {
            case 0: // HEADER 1BYTE
                if ( HEADER_1BYTE == data ) {
                    _step = 1;
                    _checksum = 0; 
                    _payload_counter = 0;
                }
                break;

            case 1: // HEADER 2BYTE
                if ( HEADER_2BYTE == data ) {
                    _step++;
                } else {
                    _step = 0;
                }
                break;

            case 2: // HEADER 3BYTE
                if ( HEADER_3BYTE == data ) {
                    _step++;
                } else {
                    _step = 0;
                }
                break;

            case 3: // Size of payload
                _payload_length = data;
                _step++;
                break;

            case 4: // Parse payload
                
                if ( _payload_counter < sizeof(_buffer) ) {
                    _buffer[_payload_counter] = data;
                    _checksum += data;
                }
                if ( ++_payload_counter == _payload_length ) {
                    _step++;
                }       
                break;

            case 5: // body checksum byte1
                if ( _checksum != data ) {
                    _step = 0;
                    _checksum = 0;
                    break;
                }
                _step++;
                break;

            case 6: // body checksum byte1
                if ( _checksum != data ) {
                    _step = 0;
                    _checksum = 0;
                    break;
                }
                _step++;
                break;

            case 7: // body checksum byte1
                if ( _checksum != data ) {
                    _step = 0;
                    _checksum = 0;
                    break;
                }
                _step++;
                break;

            case 8: // body checksum byte2
                _step = 0;
                if ( _checksum != data ) {
                    break;
                }
                parse_body();
                return true;
        }
    }
    return false;
}
