#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include "AP_BattMonitor_Params.h"

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
  #define DEFAULT_LOW_BATTERY_VOLTAGE 10.5f
#else
  #define DEFAULT_LOW_BATTERY_VOLTAGE 0.0f
#endif // APM_BUILD_TYPE(APM_BUILD_ArduCopter)

const AP_Param::GroupInfo AP_BattMonitor_Params::var_info[] = {
    // @Param: MONITOR
    // @DisplayName: Battery monitoring
    // @Description: Controls enabling monitoring of the battery's voltage and current
    // @Values: 0:Disabled,3:Analog Voltage Only,4:Analog Voltage and Current,5:Solo,6:Bebop,7:SMBus-Maxell,8:UAVCAN-BatteryInfo,9:BLHeli ESC
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("MONITOR", 1, AP_BattMonitor_Params, _type, BattMonitor_TYPE_GOV, AP_PARAM_FLAG_ENABLE),

    // @Param: CAPACITY
    // @DisplayName: Battery capacity
    // @Description: Capacity of the battery in mAh when full
    // @Units: mAh
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("CAPACITY", 7, AP_BattMonitor_Params, _pack_capacity, 3300),

    // @Param: LOW_TIMER
    // @DisplayName: Low voltage timeout
    // @Description: This is the timeout in seconds before a low voltage event will be triggered. For aircraft with low C batteries it may be necessary to raise this in order to cope with low voltage on long takeoffs. A value of zero disables low voltage errors.
    // @Units: s
    // @Increment: 1
    // @Range: 0 120
    // @User: Advanced
    AP_GROUPINFO("LOW_TIMER", 10, AP_BattMonitor_Params, _low_voltage_timeout, 10),

    // @Param: FS_VOLTSRC
    // @DisplayName: Failsafe voltage source
    // @Description: Voltage type used for detection of low voltage event
    // @Values: 0:Raw Voltage, 1:Sag Compensated Voltage
    // @User: Advanced
    AP_GROUPINFO("FS_VOLTSRC", 11, AP_BattMonitor_Params, _failsafe_voltage_source, BattMonitor_LowVoltageSource_Raw),

    // @Param: LOW_VOLT
    // @DisplayName: Low battery voltage
    // @Description: Battery voltage that triggers a low battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the @PREFIX@LOW_TIMER parameter then the vehicle will perform the failsafe specified by the @PREFIX@FS_LOW_ACT parameter.
    // @Units: V
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("LOW_VOLT", 12, AP_BattMonitor_Params, _low_voltage, DEFAULT_LOW_BATTERY_VOLTAGE),

    // @Param: LOW_MAH
    // @DisplayName: Low battery capacity
    // @Description: Battery capacity at which the low battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the @PREFIX@FS_LOW_ACT parameter.
    // @Units: mAh
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("LOW_MAH", 13, AP_BattMonitor_Params, _low_capacity, 0),

    // @Param: CRT_VOLT
    // @DisplayName: Critical battery voltage
    // @Description: Battery voltage that triggers a critical battery failsafe. Set to 0 to disable. If the battery voltage drops below this voltage continuously for more then the period specified by the @PREFIX@LOW_TIMER parameter then the vehicle will perform the failsafe specified by the @PREFIX@FS_CRT_ACT parameter.
    // @Units: V
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("CRT_VOLT", 14, AP_BattMonitor_Params, _critical_voltage, 0),

    // @Param: CRT_MAH
    // @DisplayName: Battery critical capacity
    // @Description: Battery capacity at which the critical battery failsafe is triggered. Set to 0 to disable battery remaining failsafe. If the battery capacity drops below this level the vehicle will perform the failsafe specified by the @PREFIX@_FS_CRT_ACT parameter.
    // @Units: mAh
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("CRT_MAH", 15, AP_BattMonitor_Params, _critical_capacity, 0),

    // @Param: FS_LOW_ACT
    // @DisplayName: Low battery failsafe action
    // @Description: What action the vehicle should perform if it hits a low battery failsafe
    // @Values{Plane}: 0:None,1:RTL,2:Land,3:Terminate
    // @Values{Copter}: 0:None,1:Land,2:RTL,3:SmartRTL,4:SmartRTL or Land,5:Terminate
    // @Values{Sub}: 0:None,2:Disarm,3:Enter surface mode
    // @Values{Rover}: 0:None,1:RTL,2:Hold,3:SmartRTL,4:SmartRTL or Hold,5:Terminate
    // @Values{Tracker}: 0:None
    // @User: Standard
    AP_GROUPINFO("FS_LOW_ACT", 16, AP_BattMonitor_Params, _failsafe_low_action, 0),

    // @Param: FS_CRT_ACT
    // @DisplayName: Critical battery failsafe action
    // @Description: What action the vehicle should perform if it hits a critical battery failsafe
    // @Values{Plane}: 0:None,1:RTL,2:Land,3:Terminate
    // @Values{Copter}: 0:None,1:Land,2:RTL,3:SmartRTL,4:SmartRTL or Land,5:Terminate
    // @Values{Sub}: 0:None,2:Disarm,3:Enter surface mode
    // @Values{Rover}: 0:None,1:RTL,2:Hold,3:SmartRTL,4:SmartRTL or Hold,5:Terminate
    // @Values{Tracker}: 0:None
    // @User: Standard
    AP_GROUPINFO("FS_CRT_ACT", 17, AP_BattMonitor_Params, _failsafe_critical_action, 0),

    AP_GROUPEND

};

AP_BattMonitor_Params::AP_BattMonitor_Params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}
