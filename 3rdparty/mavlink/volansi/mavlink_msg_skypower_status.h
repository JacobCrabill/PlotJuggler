#pragma once
// MESSAGE SKYPOWER_STATUS PACKING

#define MAVLINK_MSG_ID_SKYPOWER_STATUS 226


typedef struct __mavlink_skypower_status_t {
 float operating_time_s; /*< [s] Engine Operational Time (s)*/
 uint16_t engine_speed; /*< [rpm] Engine RPM*/
 uint16_t target_load; /*< [%*10] Engine Target Load (%*10)*/
 uint16_t current_load; /*< [%*10] Engine Current Load (%*10)*/
 uint16_t current_torque; /*< [Nm*10] Engine Current Torque (Nm*10)*/
 uint16_t current_power; /*< [kW*100] Engine Current Power (kW*100)*/
 uint16_t max_load; /*< [%*10] Engine Maximum Load (%*10)*/
 uint16_t rev_count; /*<  Revolutions Counter*/
 uint16_t fuel_flow; /*< [mL/hr] Injection Fuel Consumption (mL/hr)*/
 uint16_t bsfc; /*< [g/kW-hr] Brake-Specific Fuel Consumption (g/kW-hr)*/
 uint16_t injection_valve_load; /*< [%*10] Injection Valve Load (%*10)*/
 uint16_t red_factor; /*< [--] red factor(?)*/
 int16_t cyl_head_temp_1; /*< [degC*10] Cylinder Head Temp 1 (degC*10)*/
 int16_t cyl_head_temp_2; /*< [degC*10] Cylinder Head Temp 2 (degC*10)*/
 int16_t exhaust_temp_1; /*< [degC*10] Exhaust Temp 1 (degC*10)*/
 int16_t exhaust_temp_2; /*< [degC*10] Exhaust Temp 2 (degC*10)*/
 int16_t msl_cyl_head_temp_1; /*< [degC*10] MSL Cylinder Head Temp 1 (degC*10)*/
 int16_t msl_cyl_head_temp_2; /*< [degC*10] MSL Cylinder Head Temp 2 (degC*10)*/
 int16_t msl_exhaust_temp_1; /*< [degC*10] MSL Exhaust Temp 1 (degC*10)*/
 int16_t msl_exhaust_temp_2; /*< [degC*10] MSL Exhaust Temp 2 (degC*10)*/
 int16_t air_temp; /*< [degC*10] Air Temperature (degC*10)*/
 int16_t ecu_temp; /*< [degC*10] ECU Temperature (degC*10)*/
 uint16_t altitude; /*< [m] Current Altitude (m)*/
 uint16_t isa_altitude; /*< [m] ISA Altitude (m)*/
 uint16_t density_altitude; /*< [m] Density Altitude (m)*/
 uint16_t pressure_altitude; /*< [m] Pressure Altitude (m)*/
 uint16_t ambient_pressure; /*< [Pa] Ambient Pressure (Pa)*/
 uint16_t sfc_map; /*< [g/kW-hr] sfc map(?)*/
 uint16_t icao_fuelflow; /*< [mL/hr] ICAO-Corrected Injection Fuel Consumption (mL/hr)*/
 uint16_t icao_power; /*< [kW*100] ICAO-Corrected Power (kW*100)*/
 uint16_t icao_torque; /*< [Nm*10] ICAO-Corrected Torque (Nm*10)*/
 int16_t ignition_angle; /*< [degCrank*10] Ingnition angle(?)*/
 int16_t ignition_gap; /*< [deg*10] Ingnition gap(?)*/
 uint16_t throttle_angle; /*< [degCrank*10] Throttle angle(?)*/
 uint16_t injection_angle; /*< [degCrank*10] Injection angle(?)*/
 uint16_t injection_time; /*< [us] Injection time(?)*/
 uint16_t sensor_error_flags; /*< [m] Sensor Error Flags Bitmask*/
 uint16_t thermal_limit_flags; /*< [m] Thermal Limit Flags Bitmask*/
 uint16_t eng_supply_voltage; /*< [V*10] Engine Supply Voltage (V*10)*/
 int16_t gen_current_out; /*< [A] Starter/Generator Current Output (A)*/
 int16_t gen_phase_current; /*< [A*100] Starter/Generator Phase Current (A*100)*/
 uint16_t gen_supply_voltage; /*< [V*100] Starter/Generator Supply Voltage (V*100)*/
 uint16_t gen_bridge_voltage; /*< [V*100] Starter/Generator Bridge Voltage (V*100)*/
 int16_t gen_battery_current; /*< [A*100] Starter/Generator Battery Current (A*100)*/
 int16_t gen_motor_speed; /*< [rpm] Starter/Generator Motor Speed*/
 int16_t gen_temp; /*< [degC*10] Starter/Generator Temperature (deg)*/
 int16_t gen_controller_temp; /*< [degC*10] Starter/Generator Controller Temperature (deg)*/
} mavlink_skypower_status_t;

#define MAVLINK_MSG_ID_SKYPOWER_STATUS_LEN 96
#define MAVLINK_MSG_ID_SKYPOWER_STATUS_MIN_LEN 96
#define MAVLINK_MSG_ID_226_LEN 96
#define MAVLINK_MSG_ID_226_MIN_LEN 96

#define MAVLINK_MSG_ID_SKYPOWER_STATUS_CRC 239
#define MAVLINK_MSG_ID_226_CRC 239



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SKYPOWER_STATUS { \
    226, \
    "SKYPOWER_STATUS", \
    47, \
    {  { "engine_speed", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_skypower_status_t, engine_speed) }, \
         { "target_load", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_skypower_status_t, target_load) }, \
         { "current_load", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_skypower_status_t, current_load) }, \
         { "current_torque", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_skypower_status_t, current_torque) }, \
         { "current_power", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_skypower_status_t, current_power) }, \
         { "max_load", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_skypower_status_t, max_load) }, \
         { "rev_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_skypower_status_t, rev_count) }, \
         { "operating_time_s", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_skypower_status_t, operating_time_s) }, \
         { "fuel_flow", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_skypower_status_t, fuel_flow) }, \
         { "bsfc", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_skypower_status_t, bsfc) }, \
         { "injection_valve_load", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_skypower_status_t, injection_valve_load) }, \
         { "red_factor", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_skypower_status_t, red_factor) }, \
         { "cyl_head_temp_1", NULL, MAVLINK_TYPE_INT16_T, 0, 26, offsetof(mavlink_skypower_status_t, cyl_head_temp_1) }, \
         { "cyl_head_temp_2", NULL, MAVLINK_TYPE_INT16_T, 0, 28, offsetof(mavlink_skypower_status_t, cyl_head_temp_2) }, \
         { "exhaust_temp_1", NULL, MAVLINK_TYPE_INT16_T, 0, 30, offsetof(mavlink_skypower_status_t, exhaust_temp_1) }, \
         { "exhaust_temp_2", NULL, MAVLINK_TYPE_INT16_T, 0, 32, offsetof(mavlink_skypower_status_t, exhaust_temp_2) }, \
         { "msl_cyl_head_temp_1", NULL, MAVLINK_TYPE_INT16_T, 0, 34, offsetof(mavlink_skypower_status_t, msl_cyl_head_temp_1) }, \
         { "msl_cyl_head_temp_2", NULL, MAVLINK_TYPE_INT16_T, 0, 36, offsetof(mavlink_skypower_status_t, msl_cyl_head_temp_2) }, \
         { "msl_exhaust_temp_1", NULL, MAVLINK_TYPE_INT16_T, 0, 38, offsetof(mavlink_skypower_status_t, msl_exhaust_temp_1) }, \
         { "msl_exhaust_temp_2", NULL, MAVLINK_TYPE_INT16_T, 0, 40, offsetof(mavlink_skypower_status_t, msl_exhaust_temp_2) }, \
         { "air_temp", NULL, MAVLINK_TYPE_INT16_T, 0, 42, offsetof(mavlink_skypower_status_t, air_temp) }, \
         { "ecu_temp", NULL, MAVLINK_TYPE_INT16_T, 0, 44, offsetof(mavlink_skypower_status_t, ecu_temp) }, \
         { "altitude", NULL, MAVLINK_TYPE_UINT16_T, 0, 46, offsetof(mavlink_skypower_status_t, altitude) }, \
         { "isa_altitude", NULL, MAVLINK_TYPE_UINT16_T, 0, 48, offsetof(mavlink_skypower_status_t, isa_altitude) }, \
         { "density_altitude", NULL, MAVLINK_TYPE_UINT16_T, 0, 50, offsetof(mavlink_skypower_status_t, density_altitude) }, \
         { "pressure_altitude", NULL, MAVLINK_TYPE_UINT16_T, 0, 52, offsetof(mavlink_skypower_status_t, pressure_altitude) }, \
         { "ambient_pressure", NULL, MAVLINK_TYPE_UINT16_T, 0, 54, offsetof(mavlink_skypower_status_t, ambient_pressure) }, \
         { "sfc_map", NULL, MAVLINK_TYPE_UINT16_T, 0, 56, offsetof(mavlink_skypower_status_t, sfc_map) }, \
         { "icao_fuelflow", NULL, MAVLINK_TYPE_UINT16_T, 0, 58, offsetof(mavlink_skypower_status_t, icao_fuelflow) }, \
         { "icao_power", NULL, MAVLINK_TYPE_UINT16_T, 0, 60, offsetof(mavlink_skypower_status_t, icao_power) }, \
         { "icao_torque", NULL, MAVLINK_TYPE_UINT16_T, 0, 62, offsetof(mavlink_skypower_status_t, icao_torque) }, \
         { "ignition_angle", NULL, MAVLINK_TYPE_INT16_T, 0, 64, offsetof(mavlink_skypower_status_t, ignition_angle) }, \
         { "ignition_gap", NULL, MAVLINK_TYPE_INT16_T, 0, 66, offsetof(mavlink_skypower_status_t, ignition_gap) }, \
         { "throttle_angle", NULL, MAVLINK_TYPE_UINT16_T, 0, 68, offsetof(mavlink_skypower_status_t, throttle_angle) }, \
         { "injection_angle", NULL, MAVLINK_TYPE_UINT16_T, 0, 70, offsetof(mavlink_skypower_status_t, injection_angle) }, \
         { "injection_time", NULL, MAVLINK_TYPE_UINT16_T, 0, 72, offsetof(mavlink_skypower_status_t, injection_time) }, \
         { "sensor_error_flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 74, offsetof(mavlink_skypower_status_t, sensor_error_flags) }, \
         { "thermal_limit_flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 76, offsetof(mavlink_skypower_status_t, thermal_limit_flags) }, \
         { "eng_supply_voltage", NULL, MAVLINK_TYPE_UINT16_T, 0, 78, offsetof(mavlink_skypower_status_t, eng_supply_voltage) }, \
         { "gen_current_out", NULL, MAVLINK_TYPE_INT16_T, 0, 80, offsetof(mavlink_skypower_status_t, gen_current_out) }, \
         { "gen_phase_current", NULL, MAVLINK_TYPE_INT16_T, 0, 82, offsetof(mavlink_skypower_status_t, gen_phase_current) }, \
         { "gen_supply_voltage", NULL, MAVLINK_TYPE_UINT16_T, 0, 84, offsetof(mavlink_skypower_status_t, gen_supply_voltage) }, \
         { "gen_bridge_voltage", NULL, MAVLINK_TYPE_UINT16_T, 0, 86, offsetof(mavlink_skypower_status_t, gen_bridge_voltage) }, \
         { "gen_battery_current", NULL, MAVLINK_TYPE_INT16_T, 0, 88, offsetof(mavlink_skypower_status_t, gen_battery_current) }, \
         { "gen_motor_speed", NULL, MAVLINK_TYPE_INT16_T, 0, 90, offsetof(mavlink_skypower_status_t, gen_motor_speed) }, \
         { "gen_temp", NULL, MAVLINK_TYPE_INT16_T, 0, 92, offsetof(mavlink_skypower_status_t, gen_temp) }, \
         { "gen_controller_temp", NULL, MAVLINK_TYPE_INT16_T, 0, 94, offsetof(mavlink_skypower_status_t, gen_controller_temp) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SKYPOWER_STATUS { \
    "SKYPOWER_STATUS", \
    47, \
    {  { "engine_speed", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_skypower_status_t, engine_speed) }, \
         { "target_load", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_skypower_status_t, target_load) }, \
         { "current_load", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_skypower_status_t, current_load) }, \
         { "current_torque", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_skypower_status_t, current_torque) }, \
         { "current_power", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_skypower_status_t, current_power) }, \
         { "max_load", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_skypower_status_t, max_load) }, \
         { "rev_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_skypower_status_t, rev_count) }, \
         { "operating_time_s", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_skypower_status_t, operating_time_s) }, \
         { "fuel_flow", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_skypower_status_t, fuel_flow) }, \
         { "bsfc", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_skypower_status_t, bsfc) }, \
         { "injection_valve_load", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_skypower_status_t, injection_valve_load) }, \
         { "red_factor", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_skypower_status_t, red_factor) }, \
         { "cyl_head_temp_1", NULL, MAVLINK_TYPE_INT16_T, 0, 26, offsetof(mavlink_skypower_status_t, cyl_head_temp_1) }, \
         { "cyl_head_temp_2", NULL, MAVLINK_TYPE_INT16_T, 0, 28, offsetof(mavlink_skypower_status_t, cyl_head_temp_2) }, \
         { "exhaust_temp_1", NULL, MAVLINK_TYPE_INT16_T, 0, 30, offsetof(mavlink_skypower_status_t, exhaust_temp_1) }, \
         { "exhaust_temp_2", NULL, MAVLINK_TYPE_INT16_T, 0, 32, offsetof(mavlink_skypower_status_t, exhaust_temp_2) }, \
         { "msl_cyl_head_temp_1", NULL, MAVLINK_TYPE_INT16_T, 0, 34, offsetof(mavlink_skypower_status_t, msl_cyl_head_temp_1) }, \
         { "msl_cyl_head_temp_2", NULL, MAVLINK_TYPE_INT16_T, 0, 36, offsetof(mavlink_skypower_status_t, msl_cyl_head_temp_2) }, \
         { "msl_exhaust_temp_1", NULL, MAVLINK_TYPE_INT16_T, 0, 38, offsetof(mavlink_skypower_status_t, msl_exhaust_temp_1) }, \
         { "msl_exhaust_temp_2", NULL, MAVLINK_TYPE_INT16_T, 0, 40, offsetof(mavlink_skypower_status_t, msl_exhaust_temp_2) }, \
         { "air_temp", NULL, MAVLINK_TYPE_INT16_T, 0, 42, offsetof(mavlink_skypower_status_t, air_temp) }, \
         { "ecu_temp", NULL, MAVLINK_TYPE_INT16_T, 0, 44, offsetof(mavlink_skypower_status_t, ecu_temp) }, \
         { "altitude", NULL, MAVLINK_TYPE_UINT16_T, 0, 46, offsetof(mavlink_skypower_status_t, altitude) }, \
         { "isa_altitude", NULL, MAVLINK_TYPE_UINT16_T, 0, 48, offsetof(mavlink_skypower_status_t, isa_altitude) }, \
         { "density_altitude", NULL, MAVLINK_TYPE_UINT16_T, 0, 50, offsetof(mavlink_skypower_status_t, density_altitude) }, \
         { "pressure_altitude", NULL, MAVLINK_TYPE_UINT16_T, 0, 52, offsetof(mavlink_skypower_status_t, pressure_altitude) }, \
         { "ambient_pressure", NULL, MAVLINK_TYPE_UINT16_T, 0, 54, offsetof(mavlink_skypower_status_t, ambient_pressure) }, \
         { "sfc_map", NULL, MAVLINK_TYPE_UINT16_T, 0, 56, offsetof(mavlink_skypower_status_t, sfc_map) }, \
         { "icao_fuelflow", NULL, MAVLINK_TYPE_UINT16_T, 0, 58, offsetof(mavlink_skypower_status_t, icao_fuelflow) }, \
         { "icao_power", NULL, MAVLINK_TYPE_UINT16_T, 0, 60, offsetof(mavlink_skypower_status_t, icao_power) }, \
         { "icao_torque", NULL, MAVLINK_TYPE_UINT16_T, 0, 62, offsetof(mavlink_skypower_status_t, icao_torque) }, \
         { "ignition_angle", NULL, MAVLINK_TYPE_INT16_T, 0, 64, offsetof(mavlink_skypower_status_t, ignition_angle) }, \
         { "ignition_gap", NULL, MAVLINK_TYPE_INT16_T, 0, 66, offsetof(mavlink_skypower_status_t, ignition_gap) }, \
         { "throttle_angle", NULL, MAVLINK_TYPE_UINT16_T, 0, 68, offsetof(mavlink_skypower_status_t, throttle_angle) }, \
         { "injection_angle", NULL, MAVLINK_TYPE_UINT16_T, 0, 70, offsetof(mavlink_skypower_status_t, injection_angle) }, \
         { "injection_time", NULL, MAVLINK_TYPE_UINT16_T, 0, 72, offsetof(mavlink_skypower_status_t, injection_time) }, \
         { "sensor_error_flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 74, offsetof(mavlink_skypower_status_t, sensor_error_flags) }, \
         { "thermal_limit_flags", NULL, MAVLINK_TYPE_UINT16_T, 0, 76, offsetof(mavlink_skypower_status_t, thermal_limit_flags) }, \
         { "eng_supply_voltage", NULL, MAVLINK_TYPE_UINT16_T, 0, 78, offsetof(mavlink_skypower_status_t, eng_supply_voltage) }, \
         { "gen_current_out", NULL, MAVLINK_TYPE_INT16_T, 0, 80, offsetof(mavlink_skypower_status_t, gen_current_out) }, \
         { "gen_phase_current", NULL, MAVLINK_TYPE_INT16_T, 0, 82, offsetof(mavlink_skypower_status_t, gen_phase_current) }, \
         { "gen_supply_voltage", NULL, MAVLINK_TYPE_UINT16_T, 0, 84, offsetof(mavlink_skypower_status_t, gen_supply_voltage) }, \
         { "gen_bridge_voltage", NULL, MAVLINK_TYPE_UINT16_T, 0, 86, offsetof(mavlink_skypower_status_t, gen_bridge_voltage) }, \
         { "gen_battery_current", NULL, MAVLINK_TYPE_INT16_T, 0, 88, offsetof(mavlink_skypower_status_t, gen_battery_current) }, \
         { "gen_motor_speed", NULL, MAVLINK_TYPE_INT16_T, 0, 90, offsetof(mavlink_skypower_status_t, gen_motor_speed) }, \
         { "gen_temp", NULL, MAVLINK_TYPE_INT16_T, 0, 92, offsetof(mavlink_skypower_status_t, gen_temp) }, \
         { "gen_controller_temp", NULL, MAVLINK_TYPE_INT16_T, 0, 94, offsetof(mavlink_skypower_status_t, gen_controller_temp) }, \
         } \
}
#endif

/**
 * @brief Pack a skypower_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param engine_speed [rpm] Engine RPM
 * @param target_load [%*10] Engine Target Load (%*10)
 * @param current_load [%*10] Engine Current Load (%*10)
 * @param current_torque [Nm*10] Engine Current Torque (Nm*10)
 * @param current_power [kW*100] Engine Current Power (kW*100)
 * @param max_load [%*10] Engine Maximum Load (%*10)
 * @param rev_count  Revolutions Counter
 * @param operating_time_s [s] Engine Operational Time (s)
 * @param fuel_flow [mL/hr] Injection Fuel Consumption (mL/hr)
 * @param bsfc [g/kW-hr] Brake-Specific Fuel Consumption (g/kW-hr)
 * @param injection_valve_load [%*10] Injection Valve Load (%*10)
 * @param red_factor [--] red factor(?)
 * @param cyl_head_temp_1 [degC*10] Cylinder Head Temp 1 (degC*10)
 * @param cyl_head_temp_2 [degC*10] Cylinder Head Temp 2 (degC*10)
 * @param exhaust_temp_1 [degC*10] Exhaust Temp 1 (degC*10)
 * @param exhaust_temp_2 [degC*10] Exhaust Temp 2 (degC*10)
 * @param msl_cyl_head_temp_1 [degC*10] MSL Cylinder Head Temp 1 (degC*10)
 * @param msl_cyl_head_temp_2 [degC*10] MSL Cylinder Head Temp 2 (degC*10)
 * @param msl_exhaust_temp_1 [degC*10] MSL Exhaust Temp 1 (degC*10)
 * @param msl_exhaust_temp_2 [degC*10] MSL Exhaust Temp 2 (degC*10)
 * @param air_temp [degC*10] Air Temperature (degC*10)
 * @param ecu_temp [degC*10] ECU Temperature (degC*10)
 * @param altitude [m] Current Altitude (m)
 * @param isa_altitude [m] ISA Altitude (m)
 * @param density_altitude [m] Density Altitude (m)
 * @param pressure_altitude [m] Pressure Altitude (m)
 * @param ambient_pressure [Pa] Ambient Pressure (Pa)
 * @param sfc_map [g/kW-hr] sfc map(?)
 * @param icao_fuelflow [mL/hr] ICAO-Corrected Injection Fuel Consumption (mL/hr)
 * @param icao_power [kW*100] ICAO-Corrected Power (kW*100)
 * @param icao_torque [Nm*10] ICAO-Corrected Torque (Nm*10)
 * @param ignition_angle [degCrank*10] Ingnition angle(?)
 * @param ignition_gap [deg*10] Ingnition gap(?)
 * @param throttle_angle [degCrank*10] Throttle angle(?)
 * @param injection_angle [degCrank*10] Injection angle(?)
 * @param injection_time [us] Injection time(?)
 * @param sensor_error_flags [m] Sensor Error Flags Bitmask
 * @param thermal_limit_flags [m] Thermal Limit Flags Bitmask
 * @param eng_supply_voltage [V*10] Engine Supply Voltage (V*10)
 * @param gen_current_out [A] Starter/Generator Current Output (A)
 * @param gen_phase_current [A*100] Starter/Generator Phase Current (A*100)
 * @param gen_supply_voltage [V*100] Starter/Generator Supply Voltage (V*100)
 * @param gen_bridge_voltage [V*100] Starter/Generator Bridge Voltage (V*100)
 * @param gen_battery_current [A*100] Starter/Generator Battery Current (A*100)
 * @param gen_motor_speed [rpm] Starter/Generator Motor Speed
 * @param gen_temp [degC*10] Starter/Generator Temperature (deg)
 * @param gen_controller_temp [degC*10] Starter/Generator Controller Temperature (deg)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_skypower_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t engine_speed, uint16_t target_load, uint16_t current_load, uint16_t current_torque, uint16_t current_power, uint16_t max_load, uint16_t rev_count, float operating_time_s, uint16_t fuel_flow, uint16_t bsfc, uint16_t injection_valve_load, uint16_t red_factor, int16_t cyl_head_temp_1, int16_t cyl_head_temp_2, int16_t exhaust_temp_1, int16_t exhaust_temp_2, int16_t msl_cyl_head_temp_1, int16_t msl_cyl_head_temp_2, int16_t msl_exhaust_temp_1, int16_t msl_exhaust_temp_2, int16_t air_temp, int16_t ecu_temp, uint16_t altitude, uint16_t isa_altitude, uint16_t density_altitude, uint16_t pressure_altitude, uint16_t ambient_pressure, uint16_t sfc_map, uint16_t icao_fuelflow, uint16_t icao_power, uint16_t icao_torque, int16_t ignition_angle, int16_t ignition_gap, uint16_t throttle_angle, uint16_t injection_angle, uint16_t injection_time, uint16_t sensor_error_flags, uint16_t thermal_limit_flags, uint16_t eng_supply_voltage, int16_t gen_current_out, int16_t gen_phase_current, uint16_t gen_supply_voltage, uint16_t gen_bridge_voltage, int16_t gen_battery_current, int16_t gen_motor_speed, int16_t gen_temp, int16_t gen_controller_temp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SKYPOWER_STATUS_LEN];
    _mav_put_float(buf, 0, operating_time_s);
    _mav_put_uint16_t(buf, 4, engine_speed);
    _mav_put_uint16_t(buf, 6, target_load);
    _mav_put_uint16_t(buf, 8, current_load);
    _mav_put_uint16_t(buf, 10, current_torque);
    _mav_put_uint16_t(buf, 12, current_power);
    _mav_put_uint16_t(buf, 14, max_load);
    _mav_put_uint16_t(buf, 16, rev_count);
    _mav_put_uint16_t(buf, 18, fuel_flow);
    _mav_put_uint16_t(buf, 20, bsfc);
    _mav_put_uint16_t(buf, 22, injection_valve_load);
    _mav_put_uint16_t(buf, 24, red_factor);
    _mav_put_int16_t(buf, 26, cyl_head_temp_1);
    _mav_put_int16_t(buf, 28, cyl_head_temp_2);
    _mav_put_int16_t(buf, 30, exhaust_temp_1);
    _mav_put_int16_t(buf, 32, exhaust_temp_2);
    _mav_put_int16_t(buf, 34, msl_cyl_head_temp_1);
    _mav_put_int16_t(buf, 36, msl_cyl_head_temp_2);
    _mav_put_int16_t(buf, 38, msl_exhaust_temp_1);
    _mav_put_int16_t(buf, 40, msl_exhaust_temp_2);
    _mav_put_int16_t(buf, 42, air_temp);
    _mav_put_int16_t(buf, 44, ecu_temp);
    _mav_put_uint16_t(buf, 46, altitude);
    _mav_put_uint16_t(buf, 48, isa_altitude);
    _mav_put_uint16_t(buf, 50, density_altitude);
    _mav_put_uint16_t(buf, 52, pressure_altitude);
    _mav_put_uint16_t(buf, 54, ambient_pressure);
    _mav_put_uint16_t(buf, 56, sfc_map);
    _mav_put_uint16_t(buf, 58, icao_fuelflow);
    _mav_put_uint16_t(buf, 60, icao_power);
    _mav_put_uint16_t(buf, 62, icao_torque);
    _mav_put_int16_t(buf, 64, ignition_angle);
    _mav_put_int16_t(buf, 66, ignition_gap);
    _mav_put_uint16_t(buf, 68, throttle_angle);
    _mav_put_uint16_t(buf, 70, injection_angle);
    _mav_put_uint16_t(buf, 72, injection_time);
    _mav_put_uint16_t(buf, 74, sensor_error_flags);
    _mav_put_uint16_t(buf, 76, thermal_limit_flags);
    _mav_put_uint16_t(buf, 78, eng_supply_voltage);
    _mav_put_int16_t(buf, 80, gen_current_out);
    _mav_put_int16_t(buf, 82, gen_phase_current);
    _mav_put_uint16_t(buf, 84, gen_supply_voltage);
    _mav_put_uint16_t(buf, 86, gen_bridge_voltage);
    _mav_put_int16_t(buf, 88, gen_battery_current);
    _mav_put_int16_t(buf, 90, gen_motor_speed);
    _mav_put_int16_t(buf, 92, gen_temp);
    _mav_put_int16_t(buf, 94, gen_controller_temp);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SKYPOWER_STATUS_LEN);
#else
    mavlink_skypower_status_t packet;
    packet.operating_time_s = operating_time_s;
    packet.engine_speed = engine_speed;
    packet.target_load = target_load;
    packet.current_load = current_load;
    packet.current_torque = current_torque;
    packet.current_power = current_power;
    packet.max_load = max_load;
    packet.rev_count = rev_count;
    packet.fuel_flow = fuel_flow;
    packet.bsfc = bsfc;
    packet.injection_valve_load = injection_valve_load;
    packet.red_factor = red_factor;
    packet.cyl_head_temp_1 = cyl_head_temp_1;
    packet.cyl_head_temp_2 = cyl_head_temp_2;
    packet.exhaust_temp_1 = exhaust_temp_1;
    packet.exhaust_temp_2 = exhaust_temp_2;
    packet.msl_cyl_head_temp_1 = msl_cyl_head_temp_1;
    packet.msl_cyl_head_temp_2 = msl_cyl_head_temp_2;
    packet.msl_exhaust_temp_1 = msl_exhaust_temp_1;
    packet.msl_exhaust_temp_2 = msl_exhaust_temp_2;
    packet.air_temp = air_temp;
    packet.ecu_temp = ecu_temp;
    packet.altitude = altitude;
    packet.isa_altitude = isa_altitude;
    packet.density_altitude = density_altitude;
    packet.pressure_altitude = pressure_altitude;
    packet.ambient_pressure = ambient_pressure;
    packet.sfc_map = sfc_map;
    packet.icao_fuelflow = icao_fuelflow;
    packet.icao_power = icao_power;
    packet.icao_torque = icao_torque;
    packet.ignition_angle = ignition_angle;
    packet.ignition_gap = ignition_gap;
    packet.throttle_angle = throttle_angle;
    packet.injection_angle = injection_angle;
    packet.injection_time = injection_time;
    packet.sensor_error_flags = sensor_error_flags;
    packet.thermal_limit_flags = thermal_limit_flags;
    packet.eng_supply_voltage = eng_supply_voltage;
    packet.gen_current_out = gen_current_out;
    packet.gen_phase_current = gen_phase_current;
    packet.gen_supply_voltage = gen_supply_voltage;
    packet.gen_bridge_voltage = gen_bridge_voltage;
    packet.gen_battery_current = gen_battery_current;
    packet.gen_motor_speed = gen_motor_speed;
    packet.gen_temp = gen_temp;
    packet.gen_controller_temp = gen_controller_temp;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SKYPOWER_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SKYPOWER_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SKYPOWER_STATUS_MIN_LEN, MAVLINK_MSG_ID_SKYPOWER_STATUS_LEN, MAVLINK_MSG_ID_SKYPOWER_STATUS_CRC);
}

/**
 * @brief Pack a skypower_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param engine_speed [rpm] Engine RPM
 * @param target_load [%*10] Engine Target Load (%*10)
 * @param current_load [%*10] Engine Current Load (%*10)
 * @param current_torque [Nm*10] Engine Current Torque (Nm*10)
 * @param current_power [kW*100] Engine Current Power (kW*100)
 * @param max_load [%*10] Engine Maximum Load (%*10)
 * @param rev_count  Revolutions Counter
 * @param operating_time_s [s] Engine Operational Time (s)
 * @param fuel_flow [mL/hr] Injection Fuel Consumption (mL/hr)
 * @param bsfc [g/kW-hr] Brake-Specific Fuel Consumption (g/kW-hr)
 * @param injection_valve_load [%*10] Injection Valve Load (%*10)
 * @param red_factor [--] red factor(?)
 * @param cyl_head_temp_1 [degC*10] Cylinder Head Temp 1 (degC*10)
 * @param cyl_head_temp_2 [degC*10] Cylinder Head Temp 2 (degC*10)
 * @param exhaust_temp_1 [degC*10] Exhaust Temp 1 (degC*10)
 * @param exhaust_temp_2 [degC*10] Exhaust Temp 2 (degC*10)
 * @param msl_cyl_head_temp_1 [degC*10] MSL Cylinder Head Temp 1 (degC*10)
 * @param msl_cyl_head_temp_2 [degC*10] MSL Cylinder Head Temp 2 (degC*10)
 * @param msl_exhaust_temp_1 [degC*10] MSL Exhaust Temp 1 (degC*10)
 * @param msl_exhaust_temp_2 [degC*10] MSL Exhaust Temp 2 (degC*10)
 * @param air_temp [degC*10] Air Temperature (degC*10)
 * @param ecu_temp [degC*10] ECU Temperature (degC*10)
 * @param altitude [m] Current Altitude (m)
 * @param isa_altitude [m] ISA Altitude (m)
 * @param density_altitude [m] Density Altitude (m)
 * @param pressure_altitude [m] Pressure Altitude (m)
 * @param ambient_pressure [Pa] Ambient Pressure (Pa)
 * @param sfc_map [g/kW-hr] sfc map(?)
 * @param icao_fuelflow [mL/hr] ICAO-Corrected Injection Fuel Consumption (mL/hr)
 * @param icao_power [kW*100] ICAO-Corrected Power (kW*100)
 * @param icao_torque [Nm*10] ICAO-Corrected Torque (Nm*10)
 * @param ignition_angle [degCrank*10] Ingnition angle(?)
 * @param ignition_gap [deg*10] Ingnition gap(?)
 * @param throttle_angle [degCrank*10] Throttle angle(?)
 * @param injection_angle [degCrank*10] Injection angle(?)
 * @param injection_time [us] Injection time(?)
 * @param sensor_error_flags [m] Sensor Error Flags Bitmask
 * @param thermal_limit_flags [m] Thermal Limit Flags Bitmask
 * @param eng_supply_voltage [V*10] Engine Supply Voltage (V*10)
 * @param gen_current_out [A] Starter/Generator Current Output (A)
 * @param gen_phase_current [A*100] Starter/Generator Phase Current (A*100)
 * @param gen_supply_voltage [V*100] Starter/Generator Supply Voltage (V*100)
 * @param gen_bridge_voltage [V*100] Starter/Generator Bridge Voltage (V*100)
 * @param gen_battery_current [A*100] Starter/Generator Battery Current (A*100)
 * @param gen_motor_speed [rpm] Starter/Generator Motor Speed
 * @param gen_temp [degC*10] Starter/Generator Temperature (deg)
 * @param gen_controller_temp [degC*10] Starter/Generator Controller Temperature (deg)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_skypower_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t engine_speed,uint16_t target_load,uint16_t current_load,uint16_t current_torque,uint16_t current_power,uint16_t max_load,uint16_t rev_count,float operating_time_s,uint16_t fuel_flow,uint16_t bsfc,uint16_t injection_valve_load,uint16_t red_factor,int16_t cyl_head_temp_1,int16_t cyl_head_temp_2,int16_t exhaust_temp_1,int16_t exhaust_temp_2,int16_t msl_cyl_head_temp_1,int16_t msl_cyl_head_temp_2,int16_t msl_exhaust_temp_1,int16_t msl_exhaust_temp_2,int16_t air_temp,int16_t ecu_temp,uint16_t altitude,uint16_t isa_altitude,uint16_t density_altitude,uint16_t pressure_altitude,uint16_t ambient_pressure,uint16_t sfc_map,uint16_t icao_fuelflow,uint16_t icao_power,uint16_t icao_torque,int16_t ignition_angle,int16_t ignition_gap,uint16_t throttle_angle,uint16_t injection_angle,uint16_t injection_time,uint16_t sensor_error_flags,uint16_t thermal_limit_flags,uint16_t eng_supply_voltage,int16_t gen_current_out,int16_t gen_phase_current,uint16_t gen_supply_voltage,uint16_t gen_bridge_voltage,int16_t gen_battery_current,int16_t gen_motor_speed,int16_t gen_temp,int16_t gen_controller_temp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SKYPOWER_STATUS_LEN];
    _mav_put_float(buf, 0, operating_time_s);
    _mav_put_uint16_t(buf, 4, engine_speed);
    _mav_put_uint16_t(buf, 6, target_load);
    _mav_put_uint16_t(buf, 8, current_load);
    _mav_put_uint16_t(buf, 10, current_torque);
    _mav_put_uint16_t(buf, 12, current_power);
    _mav_put_uint16_t(buf, 14, max_load);
    _mav_put_uint16_t(buf, 16, rev_count);
    _mav_put_uint16_t(buf, 18, fuel_flow);
    _mav_put_uint16_t(buf, 20, bsfc);
    _mav_put_uint16_t(buf, 22, injection_valve_load);
    _mav_put_uint16_t(buf, 24, red_factor);
    _mav_put_int16_t(buf, 26, cyl_head_temp_1);
    _mav_put_int16_t(buf, 28, cyl_head_temp_2);
    _mav_put_int16_t(buf, 30, exhaust_temp_1);
    _mav_put_int16_t(buf, 32, exhaust_temp_2);
    _mav_put_int16_t(buf, 34, msl_cyl_head_temp_1);
    _mav_put_int16_t(buf, 36, msl_cyl_head_temp_2);
    _mav_put_int16_t(buf, 38, msl_exhaust_temp_1);
    _mav_put_int16_t(buf, 40, msl_exhaust_temp_2);
    _mav_put_int16_t(buf, 42, air_temp);
    _mav_put_int16_t(buf, 44, ecu_temp);
    _mav_put_uint16_t(buf, 46, altitude);
    _mav_put_uint16_t(buf, 48, isa_altitude);
    _mav_put_uint16_t(buf, 50, density_altitude);
    _mav_put_uint16_t(buf, 52, pressure_altitude);
    _mav_put_uint16_t(buf, 54, ambient_pressure);
    _mav_put_uint16_t(buf, 56, sfc_map);
    _mav_put_uint16_t(buf, 58, icao_fuelflow);
    _mav_put_uint16_t(buf, 60, icao_power);
    _mav_put_uint16_t(buf, 62, icao_torque);
    _mav_put_int16_t(buf, 64, ignition_angle);
    _mav_put_int16_t(buf, 66, ignition_gap);
    _mav_put_uint16_t(buf, 68, throttle_angle);
    _mav_put_uint16_t(buf, 70, injection_angle);
    _mav_put_uint16_t(buf, 72, injection_time);
    _mav_put_uint16_t(buf, 74, sensor_error_flags);
    _mav_put_uint16_t(buf, 76, thermal_limit_flags);
    _mav_put_uint16_t(buf, 78, eng_supply_voltage);
    _mav_put_int16_t(buf, 80, gen_current_out);
    _mav_put_int16_t(buf, 82, gen_phase_current);
    _mav_put_uint16_t(buf, 84, gen_supply_voltage);
    _mav_put_uint16_t(buf, 86, gen_bridge_voltage);
    _mav_put_int16_t(buf, 88, gen_battery_current);
    _mav_put_int16_t(buf, 90, gen_motor_speed);
    _mav_put_int16_t(buf, 92, gen_temp);
    _mav_put_int16_t(buf, 94, gen_controller_temp);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SKYPOWER_STATUS_LEN);
#else
    mavlink_skypower_status_t packet;
    packet.operating_time_s = operating_time_s;
    packet.engine_speed = engine_speed;
    packet.target_load = target_load;
    packet.current_load = current_load;
    packet.current_torque = current_torque;
    packet.current_power = current_power;
    packet.max_load = max_load;
    packet.rev_count = rev_count;
    packet.fuel_flow = fuel_flow;
    packet.bsfc = bsfc;
    packet.injection_valve_load = injection_valve_load;
    packet.red_factor = red_factor;
    packet.cyl_head_temp_1 = cyl_head_temp_1;
    packet.cyl_head_temp_2 = cyl_head_temp_2;
    packet.exhaust_temp_1 = exhaust_temp_1;
    packet.exhaust_temp_2 = exhaust_temp_2;
    packet.msl_cyl_head_temp_1 = msl_cyl_head_temp_1;
    packet.msl_cyl_head_temp_2 = msl_cyl_head_temp_2;
    packet.msl_exhaust_temp_1 = msl_exhaust_temp_1;
    packet.msl_exhaust_temp_2 = msl_exhaust_temp_2;
    packet.air_temp = air_temp;
    packet.ecu_temp = ecu_temp;
    packet.altitude = altitude;
    packet.isa_altitude = isa_altitude;
    packet.density_altitude = density_altitude;
    packet.pressure_altitude = pressure_altitude;
    packet.ambient_pressure = ambient_pressure;
    packet.sfc_map = sfc_map;
    packet.icao_fuelflow = icao_fuelflow;
    packet.icao_power = icao_power;
    packet.icao_torque = icao_torque;
    packet.ignition_angle = ignition_angle;
    packet.ignition_gap = ignition_gap;
    packet.throttle_angle = throttle_angle;
    packet.injection_angle = injection_angle;
    packet.injection_time = injection_time;
    packet.sensor_error_flags = sensor_error_flags;
    packet.thermal_limit_flags = thermal_limit_flags;
    packet.eng_supply_voltage = eng_supply_voltage;
    packet.gen_current_out = gen_current_out;
    packet.gen_phase_current = gen_phase_current;
    packet.gen_supply_voltage = gen_supply_voltage;
    packet.gen_bridge_voltage = gen_bridge_voltage;
    packet.gen_battery_current = gen_battery_current;
    packet.gen_motor_speed = gen_motor_speed;
    packet.gen_temp = gen_temp;
    packet.gen_controller_temp = gen_controller_temp;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SKYPOWER_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SKYPOWER_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SKYPOWER_STATUS_MIN_LEN, MAVLINK_MSG_ID_SKYPOWER_STATUS_LEN, MAVLINK_MSG_ID_SKYPOWER_STATUS_CRC);
}

/**
 * @brief Encode a skypower_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param skypower_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_skypower_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_skypower_status_t* skypower_status)
{
    return mavlink_msg_skypower_status_pack(system_id, component_id, msg, skypower_status->engine_speed, skypower_status->target_load, skypower_status->current_load, skypower_status->current_torque, skypower_status->current_power, skypower_status->max_load, skypower_status->rev_count, skypower_status->operating_time_s, skypower_status->fuel_flow, skypower_status->bsfc, skypower_status->injection_valve_load, skypower_status->red_factor, skypower_status->cyl_head_temp_1, skypower_status->cyl_head_temp_2, skypower_status->exhaust_temp_1, skypower_status->exhaust_temp_2, skypower_status->msl_cyl_head_temp_1, skypower_status->msl_cyl_head_temp_2, skypower_status->msl_exhaust_temp_1, skypower_status->msl_exhaust_temp_2, skypower_status->air_temp, skypower_status->ecu_temp, skypower_status->altitude, skypower_status->isa_altitude, skypower_status->density_altitude, skypower_status->pressure_altitude, skypower_status->ambient_pressure, skypower_status->sfc_map, skypower_status->icao_fuelflow, skypower_status->icao_power, skypower_status->icao_torque, skypower_status->ignition_angle, skypower_status->ignition_gap, skypower_status->throttle_angle, skypower_status->injection_angle, skypower_status->injection_time, skypower_status->sensor_error_flags, skypower_status->thermal_limit_flags, skypower_status->eng_supply_voltage, skypower_status->gen_current_out, skypower_status->gen_phase_current, skypower_status->gen_supply_voltage, skypower_status->gen_bridge_voltage, skypower_status->gen_battery_current, skypower_status->gen_motor_speed, skypower_status->gen_temp, skypower_status->gen_controller_temp);
}

/**
 * @brief Encode a skypower_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param skypower_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_skypower_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_skypower_status_t* skypower_status)
{
    return mavlink_msg_skypower_status_pack_chan(system_id, component_id, chan, msg, skypower_status->engine_speed, skypower_status->target_load, skypower_status->current_load, skypower_status->current_torque, skypower_status->current_power, skypower_status->max_load, skypower_status->rev_count, skypower_status->operating_time_s, skypower_status->fuel_flow, skypower_status->bsfc, skypower_status->injection_valve_load, skypower_status->red_factor, skypower_status->cyl_head_temp_1, skypower_status->cyl_head_temp_2, skypower_status->exhaust_temp_1, skypower_status->exhaust_temp_2, skypower_status->msl_cyl_head_temp_1, skypower_status->msl_cyl_head_temp_2, skypower_status->msl_exhaust_temp_1, skypower_status->msl_exhaust_temp_2, skypower_status->air_temp, skypower_status->ecu_temp, skypower_status->altitude, skypower_status->isa_altitude, skypower_status->density_altitude, skypower_status->pressure_altitude, skypower_status->ambient_pressure, skypower_status->sfc_map, skypower_status->icao_fuelflow, skypower_status->icao_power, skypower_status->icao_torque, skypower_status->ignition_angle, skypower_status->ignition_gap, skypower_status->throttle_angle, skypower_status->injection_angle, skypower_status->injection_time, skypower_status->sensor_error_flags, skypower_status->thermal_limit_flags, skypower_status->eng_supply_voltage, skypower_status->gen_current_out, skypower_status->gen_phase_current, skypower_status->gen_supply_voltage, skypower_status->gen_bridge_voltage, skypower_status->gen_battery_current, skypower_status->gen_motor_speed, skypower_status->gen_temp, skypower_status->gen_controller_temp);
}

/**
 * @brief Send a skypower_status message
 * @param chan MAVLink channel to send the message
 *
 * @param engine_speed [rpm] Engine RPM
 * @param target_load [%*10] Engine Target Load (%*10)
 * @param current_load [%*10] Engine Current Load (%*10)
 * @param current_torque [Nm*10] Engine Current Torque (Nm*10)
 * @param current_power [kW*100] Engine Current Power (kW*100)
 * @param max_load [%*10] Engine Maximum Load (%*10)
 * @param rev_count  Revolutions Counter
 * @param operating_time_s [s] Engine Operational Time (s)
 * @param fuel_flow [mL/hr] Injection Fuel Consumption (mL/hr)
 * @param bsfc [g/kW-hr] Brake-Specific Fuel Consumption (g/kW-hr)
 * @param injection_valve_load [%*10] Injection Valve Load (%*10)
 * @param red_factor [--] red factor(?)
 * @param cyl_head_temp_1 [degC*10] Cylinder Head Temp 1 (degC*10)
 * @param cyl_head_temp_2 [degC*10] Cylinder Head Temp 2 (degC*10)
 * @param exhaust_temp_1 [degC*10] Exhaust Temp 1 (degC*10)
 * @param exhaust_temp_2 [degC*10] Exhaust Temp 2 (degC*10)
 * @param msl_cyl_head_temp_1 [degC*10] MSL Cylinder Head Temp 1 (degC*10)
 * @param msl_cyl_head_temp_2 [degC*10] MSL Cylinder Head Temp 2 (degC*10)
 * @param msl_exhaust_temp_1 [degC*10] MSL Exhaust Temp 1 (degC*10)
 * @param msl_exhaust_temp_2 [degC*10] MSL Exhaust Temp 2 (degC*10)
 * @param air_temp [degC*10] Air Temperature (degC*10)
 * @param ecu_temp [degC*10] ECU Temperature (degC*10)
 * @param altitude [m] Current Altitude (m)
 * @param isa_altitude [m] ISA Altitude (m)
 * @param density_altitude [m] Density Altitude (m)
 * @param pressure_altitude [m] Pressure Altitude (m)
 * @param ambient_pressure [Pa] Ambient Pressure (Pa)
 * @param sfc_map [g/kW-hr] sfc map(?)
 * @param icao_fuelflow [mL/hr] ICAO-Corrected Injection Fuel Consumption (mL/hr)
 * @param icao_power [kW*100] ICAO-Corrected Power (kW*100)
 * @param icao_torque [Nm*10] ICAO-Corrected Torque (Nm*10)
 * @param ignition_angle [degCrank*10] Ingnition angle(?)
 * @param ignition_gap [deg*10] Ingnition gap(?)
 * @param throttle_angle [degCrank*10] Throttle angle(?)
 * @param injection_angle [degCrank*10] Injection angle(?)
 * @param injection_time [us] Injection time(?)
 * @param sensor_error_flags [m] Sensor Error Flags Bitmask
 * @param thermal_limit_flags [m] Thermal Limit Flags Bitmask
 * @param eng_supply_voltage [V*10] Engine Supply Voltage (V*10)
 * @param gen_current_out [A] Starter/Generator Current Output (A)
 * @param gen_phase_current [A*100] Starter/Generator Phase Current (A*100)
 * @param gen_supply_voltage [V*100] Starter/Generator Supply Voltage (V*100)
 * @param gen_bridge_voltage [V*100] Starter/Generator Bridge Voltage (V*100)
 * @param gen_battery_current [A*100] Starter/Generator Battery Current (A*100)
 * @param gen_motor_speed [rpm] Starter/Generator Motor Speed
 * @param gen_temp [degC*10] Starter/Generator Temperature (deg)
 * @param gen_controller_temp [degC*10] Starter/Generator Controller Temperature (deg)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_skypower_status_send(mavlink_channel_t chan, uint16_t engine_speed, uint16_t target_load, uint16_t current_load, uint16_t current_torque, uint16_t current_power, uint16_t max_load, uint16_t rev_count, float operating_time_s, uint16_t fuel_flow, uint16_t bsfc, uint16_t injection_valve_load, uint16_t red_factor, int16_t cyl_head_temp_1, int16_t cyl_head_temp_2, int16_t exhaust_temp_1, int16_t exhaust_temp_2, int16_t msl_cyl_head_temp_1, int16_t msl_cyl_head_temp_2, int16_t msl_exhaust_temp_1, int16_t msl_exhaust_temp_2, int16_t air_temp, int16_t ecu_temp, uint16_t altitude, uint16_t isa_altitude, uint16_t density_altitude, uint16_t pressure_altitude, uint16_t ambient_pressure, uint16_t sfc_map, uint16_t icao_fuelflow, uint16_t icao_power, uint16_t icao_torque, int16_t ignition_angle, int16_t ignition_gap, uint16_t throttle_angle, uint16_t injection_angle, uint16_t injection_time, uint16_t sensor_error_flags, uint16_t thermal_limit_flags, uint16_t eng_supply_voltage, int16_t gen_current_out, int16_t gen_phase_current, uint16_t gen_supply_voltage, uint16_t gen_bridge_voltage, int16_t gen_battery_current, int16_t gen_motor_speed, int16_t gen_temp, int16_t gen_controller_temp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SKYPOWER_STATUS_LEN];
    _mav_put_float(buf, 0, operating_time_s);
    _mav_put_uint16_t(buf, 4, engine_speed);
    _mav_put_uint16_t(buf, 6, target_load);
    _mav_put_uint16_t(buf, 8, current_load);
    _mav_put_uint16_t(buf, 10, current_torque);
    _mav_put_uint16_t(buf, 12, current_power);
    _mav_put_uint16_t(buf, 14, max_load);
    _mav_put_uint16_t(buf, 16, rev_count);
    _mav_put_uint16_t(buf, 18, fuel_flow);
    _mav_put_uint16_t(buf, 20, bsfc);
    _mav_put_uint16_t(buf, 22, injection_valve_load);
    _mav_put_uint16_t(buf, 24, red_factor);
    _mav_put_int16_t(buf, 26, cyl_head_temp_1);
    _mav_put_int16_t(buf, 28, cyl_head_temp_2);
    _mav_put_int16_t(buf, 30, exhaust_temp_1);
    _mav_put_int16_t(buf, 32, exhaust_temp_2);
    _mav_put_int16_t(buf, 34, msl_cyl_head_temp_1);
    _mav_put_int16_t(buf, 36, msl_cyl_head_temp_2);
    _mav_put_int16_t(buf, 38, msl_exhaust_temp_1);
    _mav_put_int16_t(buf, 40, msl_exhaust_temp_2);
    _mav_put_int16_t(buf, 42, air_temp);
    _mav_put_int16_t(buf, 44, ecu_temp);
    _mav_put_uint16_t(buf, 46, altitude);
    _mav_put_uint16_t(buf, 48, isa_altitude);
    _mav_put_uint16_t(buf, 50, density_altitude);
    _mav_put_uint16_t(buf, 52, pressure_altitude);
    _mav_put_uint16_t(buf, 54, ambient_pressure);
    _mav_put_uint16_t(buf, 56, sfc_map);
    _mav_put_uint16_t(buf, 58, icao_fuelflow);
    _mav_put_uint16_t(buf, 60, icao_power);
    _mav_put_uint16_t(buf, 62, icao_torque);
    _mav_put_int16_t(buf, 64, ignition_angle);
    _mav_put_int16_t(buf, 66, ignition_gap);
    _mav_put_uint16_t(buf, 68, throttle_angle);
    _mav_put_uint16_t(buf, 70, injection_angle);
    _mav_put_uint16_t(buf, 72, injection_time);
    _mav_put_uint16_t(buf, 74, sensor_error_flags);
    _mav_put_uint16_t(buf, 76, thermal_limit_flags);
    _mav_put_uint16_t(buf, 78, eng_supply_voltage);
    _mav_put_int16_t(buf, 80, gen_current_out);
    _mav_put_int16_t(buf, 82, gen_phase_current);
    _mav_put_uint16_t(buf, 84, gen_supply_voltage);
    _mav_put_uint16_t(buf, 86, gen_bridge_voltage);
    _mav_put_int16_t(buf, 88, gen_battery_current);
    _mav_put_int16_t(buf, 90, gen_motor_speed);
    _mav_put_int16_t(buf, 92, gen_temp);
    _mav_put_int16_t(buf, 94, gen_controller_temp);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SKYPOWER_STATUS, buf, MAVLINK_MSG_ID_SKYPOWER_STATUS_MIN_LEN, MAVLINK_MSG_ID_SKYPOWER_STATUS_LEN, MAVLINK_MSG_ID_SKYPOWER_STATUS_CRC);
#else
    mavlink_skypower_status_t packet;
    packet.operating_time_s = operating_time_s;
    packet.engine_speed = engine_speed;
    packet.target_load = target_load;
    packet.current_load = current_load;
    packet.current_torque = current_torque;
    packet.current_power = current_power;
    packet.max_load = max_load;
    packet.rev_count = rev_count;
    packet.fuel_flow = fuel_flow;
    packet.bsfc = bsfc;
    packet.injection_valve_load = injection_valve_load;
    packet.red_factor = red_factor;
    packet.cyl_head_temp_1 = cyl_head_temp_1;
    packet.cyl_head_temp_2 = cyl_head_temp_2;
    packet.exhaust_temp_1 = exhaust_temp_1;
    packet.exhaust_temp_2 = exhaust_temp_2;
    packet.msl_cyl_head_temp_1 = msl_cyl_head_temp_1;
    packet.msl_cyl_head_temp_2 = msl_cyl_head_temp_2;
    packet.msl_exhaust_temp_1 = msl_exhaust_temp_1;
    packet.msl_exhaust_temp_2 = msl_exhaust_temp_2;
    packet.air_temp = air_temp;
    packet.ecu_temp = ecu_temp;
    packet.altitude = altitude;
    packet.isa_altitude = isa_altitude;
    packet.density_altitude = density_altitude;
    packet.pressure_altitude = pressure_altitude;
    packet.ambient_pressure = ambient_pressure;
    packet.sfc_map = sfc_map;
    packet.icao_fuelflow = icao_fuelflow;
    packet.icao_power = icao_power;
    packet.icao_torque = icao_torque;
    packet.ignition_angle = ignition_angle;
    packet.ignition_gap = ignition_gap;
    packet.throttle_angle = throttle_angle;
    packet.injection_angle = injection_angle;
    packet.injection_time = injection_time;
    packet.sensor_error_flags = sensor_error_flags;
    packet.thermal_limit_flags = thermal_limit_flags;
    packet.eng_supply_voltage = eng_supply_voltage;
    packet.gen_current_out = gen_current_out;
    packet.gen_phase_current = gen_phase_current;
    packet.gen_supply_voltage = gen_supply_voltage;
    packet.gen_bridge_voltage = gen_bridge_voltage;
    packet.gen_battery_current = gen_battery_current;
    packet.gen_motor_speed = gen_motor_speed;
    packet.gen_temp = gen_temp;
    packet.gen_controller_temp = gen_controller_temp;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SKYPOWER_STATUS, (const char *)&packet, MAVLINK_MSG_ID_SKYPOWER_STATUS_MIN_LEN, MAVLINK_MSG_ID_SKYPOWER_STATUS_LEN, MAVLINK_MSG_ID_SKYPOWER_STATUS_CRC);
#endif
}

/**
 * @brief Send a skypower_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_skypower_status_send_struct(mavlink_channel_t chan, const mavlink_skypower_status_t* skypower_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_skypower_status_send(chan, skypower_status->engine_speed, skypower_status->target_load, skypower_status->current_load, skypower_status->current_torque, skypower_status->current_power, skypower_status->max_load, skypower_status->rev_count, skypower_status->operating_time_s, skypower_status->fuel_flow, skypower_status->bsfc, skypower_status->injection_valve_load, skypower_status->red_factor, skypower_status->cyl_head_temp_1, skypower_status->cyl_head_temp_2, skypower_status->exhaust_temp_1, skypower_status->exhaust_temp_2, skypower_status->msl_cyl_head_temp_1, skypower_status->msl_cyl_head_temp_2, skypower_status->msl_exhaust_temp_1, skypower_status->msl_exhaust_temp_2, skypower_status->air_temp, skypower_status->ecu_temp, skypower_status->altitude, skypower_status->isa_altitude, skypower_status->density_altitude, skypower_status->pressure_altitude, skypower_status->ambient_pressure, skypower_status->sfc_map, skypower_status->icao_fuelflow, skypower_status->icao_power, skypower_status->icao_torque, skypower_status->ignition_angle, skypower_status->ignition_gap, skypower_status->throttle_angle, skypower_status->injection_angle, skypower_status->injection_time, skypower_status->sensor_error_flags, skypower_status->thermal_limit_flags, skypower_status->eng_supply_voltage, skypower_status->gen_current_out, skypower_status->gen_phase_current, skypower_status->gen_supply_voltage, skypower_status->gen_bridge_voltage, skypower_status->gen_battery_current, skypower_status->gen_motor_speed, skypower_status->gen_temp, skypower_status->gen_controller_temp);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SKYPOWER_STATUS, (const char *)skypower_status, MAVLINK_MSG_ID_SKYPOWER_STATUS_MIN_LEN, MAVLINK_MSG_ID_SKYPOWER_STATUS_LEN, MAVLINK_MSG_ID_SKYPOWER_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_SKYPOWER_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_skypower_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t engine_speed, uint16_t target_load, uint16_t current_load, uint16_t current_torque, uint16_t current_power, uint16_t max_load, uint16_t rev_count, float operating_time_s, uint16_t fuel_flow, uint16_t bsfc, uint16_t injection_valve_load, uint16_t red_factor, int16_t cyl_head_temp_1, int16_t cyl_head_temp_2, int16_t exhaust_temp_1, int16_t exhaust_temp_2, int16_t msl_cyl_head_temp_1, int16_t msl_cyl_head_temp_2, int16_t msl_exhaust_temp_1, int16_t msl_exhaust_temp_2, int16_t air_temp, int16_t ecu_temp, uint16_t altitude, uint16_t isa_altitude, uint16_t density_altitude, uint16_t pressure_altitude, uint16_t ambient_pressure, uint16_t sfc_map, uint16_t icao_fuelflow, uint16_t icao_power, uint16_t icao_torque, int16_t ignition_angle, int16_t ignition_gap, uint16_t throttle_angle, uint16_t injection_angle, uint16_t injection_time, uint16_t sensor_error_flags, uint16_t thermal_limit_flags, uint16_t eng_supply_voltage, int16_t gen_current_out, int16_t gen_phase_current, uint16_t gen_supply_voltage, uint16_t gen_bridge_voltage, int16_t gen_battery_current, int16_t gen_motor_speed, int16_t gen_temp, int16_t gen_controller_temp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, operating_time_s);
    _mav_put_uint16_t(buf, 4, engine_speed);
    _mav_put_uint16_t(buf, 6, target_load);
    _mav_put_uint16_t(buf, 8, current_load);
    _mav_put_uint16_t(buf, 10, current_torque);
    _mav_put_uint16_t(buf, 12, current_power);
    _mav_put_uint16_t(buf, 14, max_load);
    _mav_put_uint16_t(buf, 16, rev_count);
    _mav_put_uint16_t(buf, 18, fuel_flow);
    _mav_put_uint16_t(buf, 20, bsfc);
    _mav_put_uint16_t(buf, 22, injection_valve_load);
    _mav_put_uint16_t(buf, 24, red_factor);
    _mav_put_int16_t(buf, 26, cyl_head_temp_1);
    _mav_put_int16_t(buf, 28, cyl_head_temp_2);
    _mav_put_int16_t(buf, 30, exhaust_temp_1);
    _mav_put_int16_t(buf, 32, exhaust_temp_2);
    _mav_put_int16_t(buf, 34, msl_cyl_head_temp_1);
    _mav_put_int16_t(buf, 36, msl_cyl_head_temp_2);
    _mav_put_int16_t(buf, 38, msl_exhaust_temp_1);
    _mav_put_int16_t(buf, 40, msl_exhaust_temp_2);
    _mav_put_int16_t(buf, 42, air_temp);
    _mav_put_int16_t(buf, 44, ecu_temp);
    _mav_put_uint16_t(buf, 46, altitude);
    _mav_put_uint16_t(buf, 48, isa_altitude);
    _mav_put_uint16_t(buf, 50, density_altitude);
    _mav_put_uint16_t(buf, 52, pressure_altitude);
    _mav_put_uint16_t(buf, 54, ambient_pressure);
    _mav_put_uint16_t(buf, 56, sfc_map);
    _mav_put_uint16_t(buf, 58, icao_fuelflow);
    _mav_put_uint16_t(buf, 60, icao_power);
    _mav_put_uint16_t(buf, 62, icao_torque);
    _mav_put_int16_t(buf, 64, ignition_angle);
    _mav_put_int16_t(buf, 66, ignition_gap);
    _mav_put_uint16_t(buf, 68, throttle_angle);
    _mav_put_uint16_t(buf, 70, injection_angle);
    _mav_put_uint16_t(buf, 72, injection_time);
    _mav_put_uint16_t(buf, 74, sensor_error_flags);
    _mav_put_uint16_t(buf, 76, thermal_limit_flags);
    _mav_put_uint16_t(buf, 78, eng_supply_voltage);
    _mav_put_int16_t(buf, 80, gen_current_out);
    _mav_put_int16_t(buf, 82, gen_phase_current);
    _mav_put_uint16_t(buf, 84, gen_supply_voltage);
    _mav_put_uint16_t(buf, 86, gen_bridge_voltage);
    _mav_put_int16_t(buf, 88, gen_battery_current);
    _mav_put_int16_t(buf, 90, gen_motor_speed);
    _mav_put_int16_t(buf, 92, gen_temp);
    _mav_put_int16_t(buf, 94, gen_controller_temp);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SKYPOWER_STATUS, buf, MAVLINK_MSG_ID_SKYPOWER_STATUS_MIN_LEN, MAVLINK_MSG_ID_SKYPOWER_STATUS_LEN, MAVLINK_MSG_ID_SKYPOWER_STATUS_CRC);
#else
    mavlink_skypower_status_t *packet = (mavlink_skypower_status_t *)msgbuf;
    packet->operating_time_s = operating_time_s;
    packet->engine_speed = engine_speed;
    packet->target_load = target_load;
    packet->current_load = current_load;
    packet->current_torque = current_torque;
    packet->current_power = current_power;
    packet->max_load = max_load;
    packet->rev_count = rev_count;
    packet->fuel_flow = fuel_flow;
    packet->bsfc = bsfc;
    packet->injection_valve_load = injection_valve_load;
    packet->red_factor = red_factor;
    packet->cyl_head_temp_1 = cyl_head_temp_1;
    packet->cyl_head_temp_2 = cyl_head_temp_2;
    packet->exhaust_temp_1 = exhaust_temp_1;
    packet->exhaust_temp_2 = exhaust_temp_2;
    packet->msl_cyl_head_temp_1 = msl_cyl_head_temp_1;
    packet->msl_cyl_head_temp_2 = msl_cyl_head_temp_2;
    packet->msl_exhaust_temp_1 = msl_exhaust_temp_1;
    packet->msl_exhaust_temp_2 = msl_exhaust_temp_2;
    packet->air_temp = air_temp;
    packet->ecu_temp = ecu_temp;
    packet->altitude = altitude;
    packet->isa_altitude = isa_altitude;
    packet->density_altitude = density_altitude;
    packet->pressure_altitude = pressure_altitude;
    packet->ambient_pressure = ambient_pressure;
    packet->sfc_map = sfc_map;
    packet->icao_fuelflow = icao_fuelflow;
    packet->icao_power = icao_power;
    packet->icao_torque = icao_torque;
    packet->ignition_angle = ignition_angle;
    packet->ignition_gap = ignition_gap;
    packet->throttle_angle = throttle_angle;
    packet->injection_angle = injection_angle;
    packet->injection_time = injection_time;
    packet->sensor_error_flags = sensor_error_flags;
    packet->thermal_limit_flags = thermal_limit_flags;
    packet->eng_supply_voltage = eng_supply_voltage;
    packet->gen_current_out = gen_current_out;
    packet->gen_phase_current = gen_phase_current;
    packet->gen_supply_voltage = gen_supply_voltage;
    packet->gen_bridge_voltage = gen_bridge_voltage;
    packet->gen_battery_current = gen_battery_current;
    packet->gen_motor_speed = gen_motor_speed;
    packet->gen_temp = gen_temp;
    packet->gen_controller_temp = gen_controller_temp;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SKYPOWER_STATUS, (const char *)packet, MAVLINK_MSG_ID_SKYPOWER_STATUS_MIN_LEN, MAVLINK_MSG_ID_SKYPOWER_STATUS_LEN, MAVLINK_MSG_ID_SKYPOWER_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE SKYPOWER_STATUS UNPACKING


/**
 * @brief Get field engine_speed from skypower_status message
 *
 * @return [rpm] Engine RPM
 */
static inline uint16_t mavlink_msg_skypower_status_get_engine_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field target_load from skypower_status message
 *
 * @return [%*10] Engine Target Load (%*10)
 */
static inline uint16_t mavlink_msg_skypower_status_get_target_load(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field current_load from skypower_status message
 *
 * @return [%*10] Engine Current Load (%*10)
 */
static inline uint16_t mavlink_msg_skypower_status_get_current_load(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field current_torque from skypower_status message
 *
 * @return [Nm*10] Engine Current Torque (Nm*10)
 */
static inline uint16_t mavlink_msg_skypower_status_get_current_torque(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field current_power from skypower_status message
 *
 * @return [kW*100] Engine Current Power (kW*100)
 */
static inline uint16_t mavlink_msg_skypower_status_get_current_power(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field max_load from skypower_status message
 *
 * @return [%*10] Engine Maximum Load (%*10)
 */
static inline uint16_t mavlink_msg_skypower_status_get_max_load(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  14);
}

/**
 * @brief Get field rev_count from skypower_status message
 *
 * @return  Revolutions Counter
 */
static inline uint16_t mavlink_msg_skypower_status_get_rev_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field operating_time_s from skypower_status message
 *
 * @return [s] Engine Operational Time (s)
 */
static inline float mavlink_msg_skypower_status_get_operating_time_s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field fuel_flow from skypower_status message
 *
 * @return [mL/hr] Injection Fuel Consumption (mL/hr)
 */
static inline uint16_t mavlink_msg_skypower_status_get_fuel_flow(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  18);
}

/**
 * @brief Get field bsfc from skypower_status message
 *
 * @return [g/kW-hr] Brake-Specific Fuel Consumption (g/kW-hr)
 */
static inline uint16_t mavlink_msg_skypower_status_get_bsfc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field injection_valve_load from skypower_status message
 *
 * @return [%*10] Injection Valve Load (%*10)
 */
static inline uint16_t mavlink_msg_skypower_status_get_injection_valve_load(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  22);
}

/**
 * @brief Get field red_factor from skypower_status message
 *
 * @return [--] red factor(?)
 */
static inline uint16_t mavlink_msg_skypower_status_get_red_factor(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field cyl_head_temp_1 from skypower_status message
 *
 * @return [degC*10] Cylinder Head Temp 1 (degC*10)
 */
static inline int16_t mavlink_msg_skypower_status_get_cyl_head_temp_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  26);
}

/**
 * @brief Get field cyl_head_temp_2 from skypower_status message
 *
 * @return [degC*10] Cylinder Head Temp 2 (degC*10)
 */
static inline int16_t mavlink_msg_skypower_status_get_cyl_head_temp_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  28);
}

/**
 * @brief Get field exhaust_temp_1 from skypower_status message
 *
 * @return [degC*10] Exhaust Temp 1 (degC*10)
 */
static inline int16_t mavlink_msg_skypower_status_get_exhaust_temp_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  30);
}

/**
 * @brief Get field exhaust_temp_2 from skypower_status message
 *
 * @return [degC*10] Exhaust Temp 2 (degC*10)
 */
static inline int16_t mavlink_msg_skypower_status_get_exhaust_temp_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  32);
}

/**
 * @brief Get field msl_cyl_head_temp_1 from skypower_status message
 *
 * @return [degC*10] MSL Cylinder Head Temp 1 (degC*10)
 */
static inline int16_t mavlink_msg_skypower_status_get_msl_cyl_head_temp_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  34);
}

/**
 * @brief Get field msl_cyl_head_temp_2 from skypower_status message
 *
 * @return [degC*10] MSL Cylinder Head Temp 2 (degC*10)
 */
static inline int16_t mavlink_msg_skypower_status_get_msl_cyl_head_temp_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  36);
}

/**
 * @brief Get field msl_exhaust_temp_1 from skypower_status message
 *
 * @return [degC*10] MSL Exhaust Temp 1 (degC*10)
 */
static inline int16_t mavlink_msg_skypower_status_get_msl_exhaust_temp_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  38);
}

/**
 * @brief Get field msl_exhaust_temp_2 from skypower_status message
 *
 * @return [degC*10] MSL Exhaust Temp 2 (degC*10)
 */
static inline int16_t mavlink_msg_skypower_status_get_msl_exhaust_temp_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  40);
}

/**
 * @brief Get field air_temp from skypower_status message
 *
 * @return [degC*10] Air Temperature (degC*10)
 */
static inline int16_t mavlink_msg_skypower_status_get_air_temp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  42);
}

/**
 * @brief Get field ecu_temp from skypower_status message
 *
 * @return [degC*10] ECU Temperature (degC*10)
 */
static inline int16_t mavlink_msg_skypower_status_get_ecu_temp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  44);
}

/**
 * @brief Get field altitude from skypower_status message
 *
 * @return [m] Current Altitude (m)
 */
static inline uint16_t mavlink_msg_skypower_status_get_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  46);
}

/**
 * @brief Get field isa_altitude from skypower_status message
 *
 * @return [m] ISA Altitude (m)
 */
static inline uint16_t mavlink_msg_skypower_status_get_isa_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  48);
}

/**
 * @brief Get field density_altitude from skypower_status message
 *
 * @return [m] Density Altitude (m)
 */
static inline uint16_t mavlink_msg_skypower_status_get_density_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  50);
}

/**
 * @brief Get field pressure_altitude from skypower_status message
 *
 * @return [m] Pressure Altitude (m)
 */
static inline uint16_t mavlink_msg_skypower_status_get_pressure_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  52);
}

/**
 * @brief Get field ambient_pressure from skypower_status message
 *
 * @return [Pa] Ambient Pressure (Pa)
 */
static inline uint16_t mavlink_msg_skypower_status_get_ambient_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  54);
}

/**
 * @brief Get field sfc_map from skypower_status message
 *
 * @return [g/kW-hr] sfc map(?)
 */
static inline uint16_t mavlink_msg_skypower_status_get_sfc_map(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  56);
}

/**
 * @brief Get field icao_fuelflow from skypower_status message
 *
 * @return [mL/hr] ICAO-Corrected Injection Fuel Consumption (mL/hr)
 */
static inline uint16_t mavlink_msg_skypower_status_get_icao_fuelflow(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  58);
}

/**
 * @brief Get field icao_power from skypower_status message
 *
 * @return [kW*100] ICAO-Corrected Power (kW*100)
 */
static inline uint16_t mavlink_msg_skypower_status_get_icao_power(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  60);
}

/**
 * @brief Get field icao_torque from skypower_status message
 *
 * @return [Nm*10] ICAO-Corrected Torque (Nm*10)
 */
static inline uint16_t mavlink_msg_skypower_status_get_icao_torque(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  62);
}

/**
 * @brief Get field ignition_angle from skypower_status message
 *
 * @return [degCrank*10] Ingnition angle(?)
 */
static inline int16_t mavlink_msg_skypower_status_get_ignition_angle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  64);
}

/**
 * @brief Get field ignition_gap from skypower_status message
 *
 * @return [deg*10] Ingnition gap(?)
 */
static inline int16_t mavlink_msg_skypower_status_get_ignition_gap(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  66);
}

/**
 * @brief Get field throttle_angle from skypower_status message
 *
 * @return [degCrank*10] Throttle angle(?)
 */
static inline uint16_t mavlink_msg_skypower_status_get_throttle_angle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  68);
}

/**
 * @brief Get field injection_angle from skypower_status message
 *
 * @return [degCrank*10] Injection angle(?)
 */
static inline uint16_t mavlink_msg_skypower_status_get_injection_angle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  70);
}

/**
 * @brief Get field injection_time from skypower_status message
 *
 * @return [us] Injection time(?)
 */
static inline uint16_t mavlink_msg_skypower_status_get_injection_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  72);
}

/**
 * @brief Get field sensor_error_flags from skypower_status message
 *
 * @return [m] Sensor Error Flags Bitmask
 */
static inline uint16_t mavlink_msg_skypower_status_get_sensor_error_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  74);
}

/**
 * @brief Get field thermal_limit_flags from skypower_status message
 *
 * @return [m] Thermal Limit Flags Bitmask
 */
static inline uint16_t mavlink_msg_skypower_status_get_thermal_limit_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  76);
}

/**
 * @brief Get field eng_supply_voltage from skypower_status message
 *
 * @return [V*10] Engine Supply Voltage (V*10)
 */
static inline uint16_t mavlink_msg_skypower_status_get_eng_supply_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  78);
}

/**
 * @brief Get field gen_current_out from skypower_status message
 *
 * @return [A] Starter/Generator Current Output (A)
 */
static inline int16_t mavlink_msg_skypower_status_get_gen_current_out(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  80);
}

/**
 * @brief Get field gen_phase_current from skypower_status message
 *
 * @return [A*100] Starter/Generator Phase Current (A*100)
 */
static inline int16_t mavlink_msg_skypower_status_get_gen_phase_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  82);
}

/**
 * @brief Get field gen_supply_voltage from skypower_status message
 *
 * @return [V*100] Starter/Generator Supply Voltage (V*100)
 */
static inline uint16_t mavlink_msg_skypower_status_get_gen_supply_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  84);
}

/**
 * @brief Get field gen_bridge_voltage from skypower_status message
 *
 * @return [V*100] Starter/Generator Bridge Voltage (V*100)
 */
static inline uint16_t mavlink_msg_skypower_status_get_gen_bridge_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  86);
}

/**
 * @brief Get field gen_battery_current from skypower_status message
 *
 * @return [A*100] Starter/Generator Battery Current (A*100)
 */
static inline int16_t mavlink_msg_skypower_status_get_gen_battery_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  88);
}

/**
 * @brief Get field gen_motor_speed from skypower_status message
 *
 * @return [rpm] Starter/Generator Motor Speed
 */
static inline int16_t mavlink_msg_skypower_status_get_gen_motor_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  90);
}

/**
 * @brief Get field gen_temp from skypower_status message
 *
 * @return [degC*10] Starter/Generator Temperature (deg)
 */
static inline int16_t mavlink_msg_skypower_status_get_gen_temp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  92);
}

/**
 * @brief Get field gen_controller_temp from skypower_status message
 *
 * @return [degC*10] Starter/Generator Controller Temperature (deg)
 */
static inline int16_t mavlink_msg_skypower_status_get_gen_controller_temp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  94);
}

/**
 * @brief Decode a skypower_status message into a struct
 *
 * @param msg The message to decode
 * @param skypower_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_skypower_status_decode(const mavlink_message_t* msg, mavlink_skypower_status_t* skypower_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    skypower_status->operating_time_s = mavlink_msg_skypower_status_get_operating_time_s(msg);
    skypower_status->engine_speed = mavlink_msg_skypower_status_get_engine_speed(msg);
    skypower_status->target_load = mavlink_msg_skypower_status_get_target_load(msg);
    skypower_status->current_load = mavlink_msg_skypower_status_get_current_load(msg);
    skypower_status->current_torque = mavlink_msg_skypower_status_get_current_torque(msg);
    skypower_status->current_power = mavlink_msg_skypower_status_get_current_power(msg);
    skypower_status->max_load = mavlink_msg_skypower_status_get_max_load(msg);
    skypower_status->rev_count = mavlink_msg_skypower_status_get_rev_count(msg);
    skypower_status->fuel_flow = mavlink_msg_skypower_status_get_fuel_flow(msg);
    skypower_status->bsfc = mavlink_msg_skypower_status_get_bsfc(msg);
    skypower_status->injection_valve_load = mavlink_msg_skypower_status_get_injection_valve_load(msg);
    skypower_status->red_factor = mavlink_msg_skypower_status_get_red_factor(msg);
    skypower_status->cyl_head_temp_1 = mavlink_msg_skypower_status_get_cyl_head_temp_1(msg);
    skypower_status->cyl_head_temp_2 = mavlink_msg_skypower_status_get_cyl_head_temp_2(msg);
    skypower_status->exhaust_temp_1 = mavlink_msg_skypower_status_get_exhaust_temp_1(msg);
    skypower_status->exhaust_temp_2 = mavlink_msg_skypower_status_get_exhaust_temp_2(msg);
    skypower_status->msl_cyl_head_temp_1 = mavlink_msg_skypower_status_get_msl_cyl_head_temp_1(msg);
    skypower_status->msl_cyl_head_temp_2 = mavlink_msg_skypower_status_get_msl_cyl_head_temp_2(msg);
    skypower_status->msl_exhaust_temp_1 = mavlink_msg_skypower_status_get_msl_exhaust_temp_1(msg);
    skypower_status->msl_exhaust_temp_2 = mavlink_msg_skypower_status_get_msl_exhaust_temp_2(msg);
    skypower_status->air_temp = mavlink_msg_skypower_status_get_air_temp(msg);
    skypower_status->ecu_temp = mavlink_msg_skypower_status_get_ecu_temp(msg);
    skypower_status->altitude = mavlink_msg_skypower_status_get_altitude(msg);
    skypower_status->isa_altitude = mavlink_msg_skypower_status_get_isa_altitude(msg);
    skypower_status->density_altitude = mavlink_msg_skypower_status_get_density_altitude(msg);
    skypower_status->pressure_altitude = mavlink_msg_skypower_status_get_pressure_altitude(msg);
    skypower_status->ambient_pressure = mavlink_msg_skypower_status_get_ambient_pressure(msg);
    skypower_status->sfc_map = mavlink_msg_skypower_status_get_sfc_map(msg);
    skypower_status->icao_fuelflow = mavlink_msg_skypower_status_get_icao_fuelflow(msg);
    skypower_status->icao_power = mavlink_msg_skypower_status_get_icao_power(msg);
    skypower_status->icao_torque = mavlink_msg_skypower_status_get_icao_torque(msg);
    skypower_status->ignition_angle = mavlink_msg_skypower_status_get_ignition_angle(msg);
    skypower_status->ignition_gap = mavlink_msg_skypower_status_get_ignition_gap(msg);
    skypower_status->throttle_angle = mavlink_msg_skypower_status_get_throttle_angle(msg);
    skypower_status->injection_angle = mavlink_msg_skypower_status_get_injection_angle(msg);
    skypower_status->injection_time = mavlink_msg_skypower_status_get_injection_time(msg);
    skypower_status->sensor_error_flags = mavlink_msg_skypower_status_get_sensor_error_flags(msg);
    skypower_status->thermal_limit_flags = mavlink_msg_skypower_status_get_thermal_limit_flags(msg);
    skypower_status->eng_supply_voltage = mavlink_msg_skypower_status_get_eng_supply_voltage(msg);
    skypower_status->gen_current_out = mavlink_msg_skypower_status_get_gen_current_out(msg);
    skypower_status->gen_phase_current = mavlink_msg_skypower_status_get_gen_phase_current(msg);
    skypower_status->gen_supply_voltage = mavlink_msg_skypower_status_get_gen_supply_voltage(msg);
    skypower_status->gen_bridge_voltage = mavlink_msg_skypower_status_get_gen_bridge_voltage(msg);
    skypower_status->gen_battery_current = mavlink_msg_skypower_status_get_gen_battery_current(msg);
    skypower_status->gen_motor_speed = mavlink_msg_skypower_status_get_gen_motor_speed(msg);
    skypower_status->gen_temp = mavlink_msg_skypower_status_get_gen_temp(msg);
    skypower_status->gen_controller_temp = mavlink_msg_skypower_status_get_gen_controller_temp(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SKYPOWER_STATUS_LEN? msg->len : MAVLINK_MSG_ID_SKYPOWER_STATUS_LEN;
        memset(skypower_status, 0, MAVLINK_MSG_ID_SKYPOWER_STATUS_LEN);
    memcpy(skypower_status, _MAV_PAYLOAD(msg), len);
#endif
}
