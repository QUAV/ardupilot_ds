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
    }
}


// Read info from sensor. This is the method called by front end at 10 Hz

void AP_BattMonitor_Serialbatt::read()
{
    if (!_initialised) {
        return;
    }

    read_incoming();

    // Read voltage from buffer
    _state.voltage = _buffer.Batt_data.Volts;

    uint32_t tnow = AP_HAL::micros();
    float dt = tnow - _state.last_time_micros;

    // Read current from buffer
    _state.current_amps = _buffer.Batt_data.Amps_Batt;

    if (_state.last_time_micros != 0 && dt < 2000000.0f) {
            // .0002778 is 1/3600 (conversion to hours)
            float mah = _state.current_amps * dt * 0.0000002778f;
            _state.consumed_mah += mah;
            _state.consumed_wh  += 0.001f * mah * _state.voltage;
        }

        // record time
        _state.last_time_micros = tnow;
}


// Parse body of the message received. In case of future aditions, implement _comand_id, as AP_Mount libraries.

void AP_BattMonitor_Serialbatt::parse_body()
{
    // TODO!! ofset, scalar, posible power consumption, etc
    // TODO!! to cope with several current readings, probably modify AP_Battmonitor.cpp methods calling backend getters.
    _state.current_amps = _buffer.Batt_data.Amps_Batt;
    _state.voltage = _buffer.Batt_data.Volts;
}


// Read inconmming message

void AP_BattMonitor_Serialbatt::read_incoming()
{
    uint8_t data;
    int16_t numc;

    numc = _port->available();

    if (numc < 0){
        return;
    }

    // int16_t in case of a longer data structure in the future
    for (int16_t i = 0; i < numc; i++){
        data = _port->read();
        switch (_step) {
            case 0: // HEADER 1BYTE
                if ( HEADER_1BYTE == data ) {
                    _step = 1;
                    _checksum = 0; 
                    _last_command_confirmed = false;
                }
                break;

            case 1: // HEADER 2BYTE
                if ( HEADER_2BYTE == data ) {
                    _step++;
                }
                break;

            case 2: // HEADER 3BYTE
                if ( HEADER_3BYTE == data ) {
                    _step++;
                }
                break;

            case 3: // Size of payload
                _checksum = data;
                _payload_length = data;
                _step++;
                break;

            case 4: // Parse payload
                _checksum += data;
                if ( _payload_counter < sizeof(_buffer) ) {
                    _buffer[_payload_counter] = data;
                }
                if ( ++_payload_counter == _payload_length )
                    _step++;
                break;

            case 5: // body checksum byte1
                _step++;
                if ( _checksum != data ) {
                    _step = 0;
                    _checksum = 0;
                    break;
                }
                _step++;
                break;

            case 6: // body checksum byte2
                _step = 0;
                if ( _checksum != data ) {
                    break;
                }
                parse_body();
        }
    }
}
