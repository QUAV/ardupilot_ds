#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Gov.h"
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <DataFlash/DataFlash.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

AP_BattMonitor *AP_BattMonitor::_singleton;

const AP_Param::GroupInfo AP_BattMonitor::var_info[] = {
    // 0 - 18, 20- 22 used by old parameter indexes

    // @Group: _
    // @Path: AP_BattMonitor_Params.cpp
    AP_SUBGROUPINFO_FLAGS(_params[0], "_", 23, AP_BattMonitor, AP_BattMonitor_Params, AP_PARAM_FLAG_IGNORE_ENABLE),

    // @Group: 2_
    // @Path: AP_BattMonitor_Params.cpp
    AP_SUBGROUPINFO(_params[1], "2_", 24, AP_BattMonitor, AP_BattMonitor_Params),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AP_BattMonitor::AP_BattMonitor(uint32_t log_battery_bit, battery_failsafe_handler_fn_t battery_failsafe_handler_fn, const int8_t *failsafe_priorities) :
    _log_battery_bit(log_battery_bit),
    _num_instances(0),
    _battery_failsafe_handler_fn(battery_failsafe_handler_fn),
    _failsafe_priorities(failsafe_priorities)
{
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_BattMonitor must be singleton");
    }
    _singleton = this;
}

// init - instantiate the battery monitors
void
AP_BattMonitor::init(const AP_SerialManager& serial_manager)
{
    // check init has not been called before
    if (_num_instances != 0) {
        return;
    }

    _highest_failsafe_priority = INT8_MAX;

    // create each instance
    for (uint8_t instance=0; instance<AP_BATT_MONITOR_MAX_INSTANCES; instance++) {
        // clear out the cell voltages
        memset(&state[instance].cell_voltages, 0xFF, sizeof(cells));

        switch (get_type(instance)) {

            case AP_BattMonitor_Params::BattMonitor_TYPE_GOV:
                drivers[instance] = new AP_BattMonitor_Gov(*this, state[instance], _params[instance]);
                _num_instances++;
                
                break;
            case AP_BattMonitor_Params::BattMonitor_TYPE_NONE:
            default:
                break;
        }

        // call init function for each backend
        if (drivers[instance] != nullptr) {
            drivers[instance]->init(serial_manager);
        }
    }
}

// read - read the voltage and current for all instances
void
AP_BattMonitor::read()
{
    for (uint8_t i=0; i<_num_instances; i++) {
        if (drivers[i] != nullptr && _params[i].type() != AP_BattMonitor_Params::BattMonitor_TYPE_NONE) {
            drivers[i]->read();
            drivers[i]->update_resistance_estimate();
        }
    }

    if (get_type() != AP_BattMonitor_Params::BattMonitor_TYPE_NONE) {
        AP_Notify::flags.battery_voltage = voltage();
    }

    DataFlash_Class *df = DataFlash_Class::instance();
    if (df->should_log(_log_battery_bit)) {
        df->Log_Write_Current();
        df->Log_Write_Power();
    }

    check_failsafes();
}

// healthy - returns true if monitor is functioning
bool AP_BattMonitor::healthy(uint8_t instance) const {
    return instance < _num_instances && state[instance].healthy;
}

/// has_consumed_energy - returns true if battery monitor instance provides consumed energy info
bool AP_BattMonitor::has_consumed_energy(uint8_t instance) const
{
    if (instance < _num_instances && drivers[instance] != nullptr && _params[instance].type() != AP_BattMonitor_Params::BattMonitor_TYPE_NONE) {
        return drivers[instance]->has_consumed_energy();
    }

    // not monitoring current
    return false;
}

/// has_current - returns true if battery monitor instance provides current info
bool AP_BattMonitor::has_current(uint8_t instance) const
{
    if (instance < _num_instances && drivers[instance] != nullptr && _params[instance].type() != AP_BattMonitor_Params::BattMonitor_TYPE_NONE) {
        return drivers[instance]->has_current();
    }

    // not monitoring current
    return false;
}

/// voltage - returns battery voltage in volts
float AP_BattMonitor::voltage(uint8_t instance) const
{
    if (instance < _num_instances) {
        return state[instance].voltage;
    } else {
        return 0.0f;
    }
}

/// get voltage with sag removed (based on battery current draw and resistance)
/// this will always be greater than or equal to the raw voltage
float AP_BattMonitor::voltage_resting_estimate(uint8_t instance) const
{
    if (instance < _num_instances) {
        // resting voltage should always be greater than or equal to the raw voltage
        return MAX(state[instance].voltage, state[instance].voltage_resting_estimate);
    } else {
        return 0.0f;
    }
}

/// current_amps - returns the instantaneous current draw in amperes
float AP_BattMonitor::current_amps(uint8_t instance) const 
{
    if (instance < _num_instances) {
        return state[instance].current_amps;
    } else {
        return 0.0f;
    }
}

/// generator_amps - returns the current generated by motor
float AP_BattMonitor::generator_amps(uint8_t instance) const
{
    if (instance < _num_instances) {
        return state[instance].generator_amps;
    } else {
        return 0.0f;
    }
}

/// rotor_amps - 
float AP_BattMonitor::rotor_amps(uint8_t instance) const
{
    if (instance < _num_instances) {
        return state[instance].rotor_amps;
    } else {
        return 0.0f;
    }
}

/// fuel_level - returns current fuel level in the tank
float AP_BattMonitor::fuel_level(uint8_t instance) const
{
    if (instance < _num_instances) {
        return state[instance].fuel_level;
    } else {
        return 0.0f;
    }
}

/// gas_percent - returns current percentage of throttle being sent to the motor
float AP_BattMonitor::gas_percent(uint8_t instance) const
{
    if (instance < _num_instances) {
        return state[instance].gas_percent;
    } else {
        return 0.0f;
    }
}

/// consumed_mah - returns total current drawn since start-up in milliampere.hours
float AP_BattMonitor::consumed_mah(uint8_t instance) const {
    if (instance < _num_instances) {
        return state[instance].consumed_mah;
    } else {
        return 0.0f;
    }
}

/// consumed_wh - returns energy consumed since start-up in Watt.hours
float AP_BattMonitor::consumed_wh(uint8_t instance) const {
    if (instance < _num_instances) {
        return state[instance].consumed_wh;
    } else {
        return 0.0f;
    }
}

/// capacity_remaining_pct - returns the % battery capacity remaining (0 ~ 100)
uint8_t AP_BattMonitor::capacity_remaining_pct(uint8_t instance) const
{
    if (instance < _num_instances && drivers[instance] != nullptr) {
        return drivers[instance]->capacity_remaining_pct();
    } else {
        return 0;
    }
}

/// pack_capacity_mah - returns the capacity of the battery pack in mAh when the pack is full
int32_t AP_BattMonitor::pack_capacity_mah(uint8_t instance) const
{
    if (instance < AP_BATT_MONITOR_MAX_INSTANCES) {
        return _params[instance]._pack_capacity;
    } else {
        return 0;
    }
}

void AP_BattMonitor::check_failsafes(void)
{
    if (hal.util->get_soft_armed()) {
        for (uint8_t i = 0; i < _num_instances; i++) {
            const BatteryFailsafe type = check_failsafe(i);
            if (type <= state[i].failsafe) {
                continue;
            }

            int8_t action = 0;
            const char *type_str = nullptr;
            switch (type) {
                case AP_BattMonitor::BatteryFailsafe_None:
                    continue; // should not have been called in this case
                case AP_BattMonitor::BatteryFailsafe_Low:
                    action = _params[i]._failsafe_low_action;
                    type_str = "low";
                    break;
                case AP_BattMonitor::BatteryFailsafe_Critical:
                    action = _params[i]._failsafe_critical_action;
                    type_str = "critical";
                    break;
            }

            gcs().send_text(MAV_SEVERITY_WARNING, "Battery %d is %s %.2fV used %.0f mAh", i + 1, type_str,
                            (double)voltage(i), (double)consumed_mah(i));
            _has_triggered_failsafe = true;
            AP_Notify::flags.failsafe_battery = true;
            state[i].failsafe = type;

            // map the desired failsafe action to a prioritiy level
            int8_t priority = 0;
            if (_failsafe_priorities != nullptr) {
                while (_failsafe_priorities[priority] != -1) {
                    if (_failsafe_priorities[priority] == action) {
                        break;
                    }
                    priority++;
                }

            }

            // trigger failsafe if the action was equal or higher priority
            // It's valid to retrigger the same action if a different battery provoked the event
            if (priority <= _highest_failsafe_priority) {
                _battery_failsafe_handler_fn(type_str, action);
                _highest_failsafe_priority = priority;
            }
        }
    }
}

// returns the failsafe state of the battery
AP_BattMonitor::BatteryFailsafe AP_BattMonitor::check_failsafe(const uint8_t instance)
{
    // exit immediately if no monitors setup
    if (_num_instances == 0 || instance >= _num_instances) {
        return BatteryFailsafe_None;
    }

    const uint32_t now = AP_HAL::millis();

    // use voltage or sag compensated voltage
    float voltage_used;
    switch (_params[instance].failsafe_voltage_source()) {
        case AP_BattMonitor_Params::BattMonitor_LowVoltageSource_Raw:
        default:
            voltage_used = state[instance].voltage;
            break;
        case AP_BattMonitor_Params::BattMonitor_LowVoltageSource_SagCompensated:
            voltage_used = voltage_resting_estimate(instance);
            break;
    }

    // check critical battery levels
    if ((voltage_used > 0) && (_params[instance]._critical_voltage > 0) && (voltage_used < _params[instance]._critical_voltage)) {
        // this is the first time our voltage has dropped below minimum so start timer
        if (state[instance].critical_voltage_start_ms == 0) {
            state[instance].critical_voltage_start_ms = now;
        } else if (_params[instance]._low_voltage_timeout > 0 &&
                   now - state[instance].critical_voltage_start_ms > uint32_t(_params[instance]._low_voltage_timeout)*1000U) {
            return BatteryFailsafe_Critical;
        }
    } else {
        // acceptable voltage so reset timer
        state[instance].critical_voltage_start_ms = 0;
    }

    // check capacity if current monitoring is enabled
    if (has_current(instance) && (_params[instance]._critical_capacity > 0) &&
        ((_params[instance]._pack_capacity - state[instance].consumed_mah) < _params[instance]._critical_capacity)) {
        return BatteryFailsafe_Critical;
    }

    if ((voltage_used > 0) && (_params[instance]._low_voltage > 0) && (voltage_used < _params[instance]._low_voltage)) {
        // this is the first time our voltage has dropped below minimum so start timer
        if (state[instance].low_voltage_start_ms == 0) {
            state[instance].low_voltage_start_ms = now;
        } else if (_params[instance]._low_voltage_timeout > 0 &&
                   now - state[instance].low_voltage_start_ms > uint32_t(_params[instance]._low_voltage_timeout)*1000U) {
            return BatteryFailsafe_Low;
        }
    } else {
        // acceptable voltage so reset timer
        state[instance].low_voltage_start_ms = 0;
    }

    // check capacity if current monitoring is enabled
    if (has_current(instance) && (_params[instance]._low_capacity > 0) &&
        ((_params[instance]._pack_capacity - state[instance].consumed_mah) < _params[instance]._low_capacity)) {
        return BatteryFailsafe_Low;
    }

    // if we've gotten this far then battery is ok
    return BatteryFailsafe_None;
}

bool AP_BattMonitor::has_cell_voltages(const uint8_t instance) const
{
    if (instance < _num_instances && drivers[instance] != nullptr) {
        return drivers[instance]->has_cell_voltages();
    }

    return false;
}

// return the current cell voltages, returns the first monitor instances cells if the instance is out of range
const AP_BattMonitor::cells & AP_BattMonitor::get_cell_voltages(const uint8_t instance) const
{
    if (instance >= AP_BATT_MONITOR_MAX_INSTANCES) {
        return state[AP_BATT_PRIMARY_INSTANCE].cell_voltages;
    } else {
        return state[instance].cell_voltages;
    }
}

// returns true if there is a temperature reading
bool AP_BattMonitor::get_temperature(float &temperature, const uint8_t instance) const
{
    if (instance >= AP_BATT_MONITOR_MAX_INSTANCES) {
        return false;
    } else {
        temperature = state[instance].temperature;
        return (AP_HAL::millis() - state[instance].temperature_time) <= AP_BATT_MONITOR_TIMEOUT;
    }
}


namespace AP {

AP_BattMonitor &battery()
{
    return AP_BattMonitor::battery();
}

};
