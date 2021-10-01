#pragma once
// MESSAGE CURRAWONG_STATUS PACKING

#define MAVLINK_MSG_ID_CURRAWONG_STATUS 227


typedef struct __mavlink_currawong_status_t {
 float throttle; /*< [%] Throttle Position*/
 float rpm; /*< [rpm] Engine Speed*/
 float fuel_used; /*< [g] Fuel Consumed*/
 float cht; /*< [degC] Cylinder Head Temperature*/
 float barometric_pressure; /*< [kPa] Barometric Pressure*/
 float intake_manifold_pressure; /*< [kPa] Intake Manifold Pressure*/
 float iat; /*< [?] iat*/
 float fuel_press; /*< [Pa] Fuel Pressure*/
 float engine_time_s; /*< [s] Engine Run Time*/
 float input_voltage; /*< [V] Input Voltage*/
 float cpu_load; /*< [%] CPU Load*/
 float charge_temp; /*< [degC] Charge Temp*/
 float ignition_angle_1; /*< [degCrank] Ignition timing for cylinder 1 (Crank Angle degrees)*/
 float ignition_angle_2; /*< [degCrank] Ignition timing for cylinder 2 (Crank Angle degrees)*/
 float fuel_flow; /*< [g/min] Fuel Flow Rate*/
 uint16_t rpm_cmd; /*< [rpm] Commanded RPM*/
 uint16_t throttle_pulse; /*< [?] Throttle Pulse*/
 uint16_t injector_duty_cycle; /*< [?] Injection duty cycle*/
 uint8_t status; /*<  ECU Status*/
 uint8_t gov_status; /*<  Governor Status*/
} mavlink_currawong_status_t;

#define MAVLINK_MSG_ID_CURRAWONG_STATUS_LEN 68
#define MAVLINK_MSG_ID_CURRAWONG_STATUS_MIN_LEN 68
#define MAVLINK_MSG_ID_227_LEN 68
#define MAVLINK_MSG_ID_227_MIN_LEN 68

#define MAVLINK_MSG_ID_CURRAWONG_STATUS_CRC 244
#define MAVLINK_MSG_ID_227_CRC 244



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CURRAWONG_STATUS { \
    227, \
    "CURRAWONG_STATUS", \
    20, \
    {  { "throttle", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_currawong_status_t, throttle) }, \
         { "rpm", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_currawong_status_t, rpm) }, \
         { "fuel_used", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_currawong_status_t, fuel_used) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 66, offsetof(mavlink_currawong_status_t, status) }, \
         { "rpm_cmd", NULL, MAVLINK_TYPE_UINT16_T, 0, 60, offsetof(mavlink_currawong_status_t, rpm_cmd) }, \
         { "throttle_pulse", NULL, MAVLINK_TYPE_UINT16_T, 0, 62, offsetof(mavlink_currawong_status_t, throttle_pulse) }, \
         { "cht", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_currawong_status_t, cht) }, \
         { "barometric_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_currawong_status_t, barometric_pressure) }, \
         { "intake_manifold_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_currawong_status_t, intake_manifold_pressure) }, \
         { "iat", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_currawong_status_t, iat) }, \
         { "fuel_press", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_currawong_status_t, fuel_press) }, \
         { "engine_time_s", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_currawong_status_t, engine_time_s) }, \
         { "input_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_currawong_status_t, input_voltage) }, \
         { "gov_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 67, offsetof(mavlink_currawong_status_t, gov_status) }, \
         { "cpu_load", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_currawong_status_t, cpu_load) }, \
         { "charge_temp", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_currawong_status_t, charge_temp) }, \
         { "injector_duty_cycle", NULL, MAVLINK_TYPE_UINT16_T, 0, 64, offsetof(mavlink_currawong_status_t, injector_duty_cycle) }, \
         { "ignition_angle_1", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_currawong_status_t, ignition_angle_1) }, \
         { "ignition_angle_2", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_currawong_status_t, ignition_angle_2) }, \
         { "fuel_flow", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_currawong_status_t, fuel_flow) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CURRAWONG_STATUS { \
    "CURRAWONG_STATUS", \
    20, \
    {  { "throttle", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_currawong_status_t, throttle) }, \
         { "rpm", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_currawong_status_t, rpm) }, \
         { "fuel_used", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_currawong_status_t, fuel_used) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 66, offsetof(mavlink_currawong_status_t, status) }, \
         { "rpm_cmd", NULL, MAVLINK_TYPE_UINT16_T, 0, 60, offsetof(mavlink_currawong_status_t, rpm_cmd) }, \
         { "throttle_pulse", NULL, MAVLINK_TYPE_UINT16_T, 0, 62, offsetof(mavlink_currawong_status_t, throttle_pulse) }, \
         { "cht", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_currawong_status_t, cht) }, \
         { "barometric_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_currawong_status_t, barometric_pressure) }, \
         { "intake_manifold_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_currawong_status_t, intake_manifold_pressure) }, \
         { "iat", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_currawong_status_t, iat) }, \
         { "fuel_press", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_currawong_status_t, fuel_press) }, \
         { "engine_time_s", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_currawong_status_t, engine_time_s) }, \
         { "input_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_currawong_status_t, input_voltage) }, \
         { "gov_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 67, offsetof(mavlink_currawong_status_t, gov_status) }, \
         { "cpu_load", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_currawong_status_t, cpu_load) }, \
         { "charge_temp", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_currawong_status_t, charge_temp) }, \
         { "injector_duty_cycle", NULL, MAVLINK_TYPE_UINT16_T, 0, 64, offsetof(mavlink_currawong_status_t, injector_duty_cycle) }, \
         { "ignition_angle_1", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_currawong_status_t, ignition_angle_1) }, \
         { "ignition_angle_2", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_currawong_status_t, ignition_angle_2) }, \
         { "fuel_flow", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_currawong_status_t, fuel_flow) }, \
         } \
}
#endif

/**
 * @brief Pack a currawong_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param throttle [%] Throttle Position
 * @param rpm [rpm] Engine Speed
 * @param fuel_used [g] Fuel Consumed
 * @param status  ECU Status
 * @param rpm_cmd [rpm] Commanded RPM
 * @param throttle_pulse [?] Throttle Pulse
 * @param cht [degC] Cylinder Head Temperature
 * @param barometric_pressure [kPa] Barometric Pressure
 * @param intake_manifold_pressure [kPa] Intake Manifold Pressure
 * @param iat [?] iat
 * @param fuel_press [Pa] Fuel Pressure
 * @param engine_time_s [s] Engine Run Time
 * @param input_voltage [V] Input Voltage
 * @param gov_status  Governor Status
 * @param cpu_load [%] CPU Load
 * @param charge_temp [degC] Charge Temp
 * @param injector_duty_cycle [?] Injection duty cycle
 * @param ignition_angle_1 [degCrank] Ignition timing for cylinder 1 (Crank Angle degrees)
 * @param ignition_angle_2 [degCrank] Ignition timing for cylinder 2 (Crank Angle degrees)
 * @param fuel_flow [g/min] Fuel Flow Rate
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_currawong_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float throttle, float rpm, float fuel_used, uint8_t status, uint16_t rpm_cmd, uint16_t throttle_pulse, float cht, float barometric_pressure, float intake_manifold_pressure, float iat, float fuel_press, float engine_time_s, float input_voltage, uint8_t gov_status, float cpu_load, float charge_temp, uint16_t injector_duty_cycle, float ignition_angle_1, float ignition_angle_2, float fuel_flow)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CURRAWONG_STATUS_LEN];
    _mav_put_float(buf, 0, throttle);
    _mav_put_float(buf, 4, rpm);
    _mav_put_float(buf, 8, fuel_used);
    _mav_put_float(buf, 12, cht);
    _mav_put_float(buf, 16, barometric_pressure);
    _mav_put_float(buf, 20, intake_manifold_pressure);
    _mav_put_float(buf, 24, iat);
    _mav_put_float(buf, 28, fuel_press);
    _mav_put_float(buf, 32, engine_time_s);
    _mav_put_float(buf, 36, input_voltage);
    _mav_put_float(buf, 40, cpu_load);
    _mav_put_float(buf, 44, charge_temp);
    _mav_put_float(buf, 48, ignition_angle_1);
    _mav_put_float(buf, 52, ignition_angle_2);
    _mav_put_float(buf, 56, fuel_flow);
    _mav_put_uint16_t(buf, 60, rpm_cmd);
    _mav_put_uint16_t(buf, 62, throttle_pulse);
    _mav_put_uint16_t(buf, 64, injector_duty_cycle);
    _mav_put_uint8_t(buf, 66, status);
    _mav_put_uint8_t(buf, 67, gov_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CURRAWONG_STATUS_LEN);
#else
    mavlink_currawong_status_t packet;
    packet.throttle = throttle;
    packet.rpm = rpm;
    packet.fuel_used = fuel_used;
    packet.cht = cht;
    packet.barometric_pressure = barometric_pressure;
    packet.intake_manifold_pressure = intake_manifold_pressure;
    packet.iat = iat;
    packet.fuel_press = fuel_press;
    packet.engine_time_s = engine_time_s;
    packet.input_voltage = input_voltage;
    packet.cpu_load = cpu_load;
    packet.charge_temp = charge_temp;
    packet.ignition_angle_1 = ignition_angle_1;
    packet.ignition_angle_2 = ignition_angle_2;
    packet.fuel_flow = fuel_flow;
    packet.rpm_cmd = rpm_cmd;
    packet.throttle_pulse = throttle_pulse;
    packet.injector_duty_cycle = injector_duty_cycle;
    packet.status = status;
    packet.gov_status = gov_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CURRAWONG_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CURRAWONG_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CURRAWONG_STATUS_MIN_LEN, MAVLINK_MSG_ID_CURRAWONG_STATUS_LEN, MAVLINK_MSG_ID_CURRAWONG_STATUS_CRC);
}

/**
 * @brief Pack a currawong_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param throttle [%] Throttle Position
 * @param rpm [rpm] Engine Speed
 * @param fuel_used [g] Fuel Consumed
 * @param status  ECU Status
 * @param rpm_cmd [rpm] Commanded RPM
 * @param throttle_pulse [?] Throttle Pulse
 * @param cht [degC] Cylinder Head Temperature
 * @param barometric_pressure [kPa] Barometric Pressure
 * @param intake_manifold_pressure [kPa] Intake Manifold Pressure
 * @param iat [?] iat
 * @param fuel_press [Pa] Fuel Pressure
 * @param engine_time_s [s] Engine Run Time
 * @param input_voltage [V] Input Voltage
 * @param gov_status  Governor Status
 * @param cpu_load [%] CPU Load
 * @param charge_temp [degC] Charge Temp
 * @param injector_duty_cycle [?] Injection duty cycle
 * @param ignition_angle_1 [degCrank] Ignition timing for cylinder 1 (Crank Angle degrees)
 * @param ignition_angle_2 [degCrank] Ignition timing for cylinder 2 (Crank Angle degrees)
 * @param fuel_flow [g/min] Fuel Flow Rate
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_currawong_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float throttle,float rpm,float fuel_used,uint8_t status,uint16_t rpm_cmd,uint16_t throttle_pulse,float cht,float barometric_pressure,float intake_manifold_pressure,float iat,float fuel_press,float engine_time_s,float input_voltage,uint8_t gov_status,float cpu_load,float charge_temp,uint16_t injector_duty_cycle,float ignition_angle_1,float ignition_angle_2,float fuel_flow)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CURRAWONG_STATUS_LEN];
    _mav_put_float(buf, 0, throttle);
    _mav_put_float(buf, 4, rpm);
    _mav_put_float(buf, 8, fuel_used);
    _mav_put_float(buf, 12, cht);
    _mav_put_float(buf, 16, barometric_pressure);
    _mav_put_float(buf, 20, intake_manifold_pressure);
    _mav_put_float(buf, 24, iat);
    _mav_put_float(buf, 28, fuel_press);
    _mav_put_float(buf, 32, engine_time_s);
    _mav_put_float(buf, 36, input_voltage);
    _mav_put_float(buf, 40, cpu_load);
    _mav_put_float(buf, 44, charge_temp);
    _mav_put_float(buf, 48, ignition_angle_1);
    _mav_put_float(buf, 52, ignition_angle_2);
    _mav_put_float(buf, 56, fuel_flow);
    _mav_put_uint16_t(buf, 60, rpm_cmd);
    _mav_put_uint16_t(buf, 62, throttle_pulse);
    _mav_put_uint16_t(buf, 64, injector_duty_cycle);
    _mav_put_uint8_t(buf, 66, status);
    _mav_put_uint8_t(buf, 67, gov_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CURRAWONG_STATUS_LEN);
#else
    mavlink_currawong_status_t packet;
    packet.throttle = throttle;
    packet.rpm = rpm;
    packet.fuel_used = fuel_used;
    packet.cht = cht;
    packet.barometric_pressure = barometric_pressure;
    packet.intake_manifold_pressure = intake_manifold_pressure;
    packet.iat = iat;
    packet.fuel_press = fuel_press;
    packet.engine_time_s = engine_time_s;
    packet.input_voltage = input_voltage;
    packet.cpu_load = cpu_load;
    packet.charge_temp = charge_temp;
    packet.ignition_angle_1 = ignition_angle_1;
    packet.ignition_angle_2 = ignition_angle_2;
    packet.fuel_flow = fuel_flow;
    packet.rpm_cmd = rpm_cmd;
    packet.throttle_pulse = throttle_pulse;
    packet.injector_duty_cycle = injector_duty_cycle;
    packet.status = status;
    packet.gov_status = gov_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CURRAWONG_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CURRAWONG_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CURRAWONG_STATUS_MIN_LEN, MAVLINK_MSG_ID_CURRAWONG_STATUS_LEN, MAVLINK_MSG_ID_CURRAWONG_STATUS_CRC);
}

/**
 * @brief Encode a currawong_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param currawong_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_currawong_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_currawong_status_t* currawong_status)
{
    return mavlink_msg_currawong_status_pack(system_id, component_id, msg, currawong_status->throttle, currawong_status->rpm, currawong_status->fuel_used, currawong_status->status, currawong_status->rpm_cmd, currawong_status->throttle_pulse, currawong_status->cht, currawong_status->barometric_pressure, currawong_status->intake_manifold_pressure, currawong_status->iat, currawong_status->fuel_press, currawong_status->engine_time_s, currawong_status->input_voltage, currawong_status->gov_status, currawong_status->cpu_load, currawong_status->charge_temp, currawong_status->injector_duty_cycle, currawong_status->ignition_angle_1, currawong_status->ignition_angle_2, currawong_status->fuel_flow);
}

/**
 * @brief Encode a currawong_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param currawong_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_currawong_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_currawong_status_t* currawong_status)
{
    return mavlink_msg_currawong_status_pack_chan(system_id, component_id, chan, msg, currawong_status->throttle, currawong_status->rpm, currawong_status->fuel_used, currawong_status->status, currawong_status->rpm_cmd, currawong_status->throttle_pulse, currawong_status->cht, currawong_status->barometric_pressure, currawong_status->intake_manifold_pressure, currawong_status->iat, currawong_status->fuel_press, currawong_status->engine_time_s, currawong_status->input_voltage, currawong_status->gov_status, currawong_status->cpu_load, currawong_status->charge_temp, currawong_status->injector_duty_cycle, currawong_status->ignition_angle_1, currawong_status->ignition_angle_2, currawong_status->fuel_flow);
}

/**
 * @brief Send a currawong_status message
 * @param chan MAVLink channel to send the message
 *
 * @param throttle [%] Throttle Position
 * @param rpm [rpm] Engine Speed
 * @param fuel_used [g] Fuel Consumed
 * @param status  ECU Status
 * @param rpm_cmd [rpm] Commanded RPM
 * @param throttle_pulse [?] Throttle Pulse
 * @param cht [degC] Cylinder Head Temperature
 * @param barometric_pressure [kPa] Barometric Pressure
 * @param intake_manifold_pressure [kPa] Intake Manifold Pressure
 * @param iat [?] iat
 * @param fuel_press [Pa] Fuel Pressure
 * @param engine_time_s [s] Engine Run Time
 * @param input_voltage [V] Input Voltage
 * @param gov_status  Governor Status
 * @param cpu_load [%] CPU Load
 * @param charge_temp [degC] Charge Temp
 * @param injector_duty_cycle [?] Injection duty cycle
 * @param ignition_angle_1 [degCrank] Ignition timing for cylinder 1 (Crank Angle degrees)
 * @param ignition_angle_2 [degCrank] Ignition timing for cylinder 2 (Crank Angle degrees)
 * @param fuel_flow [g/min] Fuel Flow Rate
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_currawong_status_send(mavlink_channel_t chan, float throttle, float rpm, float fuel_used, uint8_t status, uint16_t rpm_cmd, uint16_t throttle_pulse, float cht, float barometric_pressure, float intake_manifold_pressure, float iat, float fuel_press, float engine_time_s, float input_voltage, uint8_t gov_status, float cpu_load, float charge_temp, uint16_t injector_duty_cycle, float ignition_angle_1, float ignition_angle_2, float fuel_flow)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CURRAWONG_STATUS_LEN];
    _mav_put_float(buf, 0, throttle);
    _mav_put_float(buf, 4, rpm);
    _mav_put_float(buf, 8, fuel_used);
    _mav_put_float(buf, 12, cht);
    _mav_put_float(buf, 16, barometric_pressure);
    _mav_put_float(buf, 20, intake_manifold_pressure);
    _mav_put_float(buf, 24, iat);
    _mav_put_float(buf, 28, fuel_press);
    _mav_put_float(buf, 32, engine_time_s);
    _mav_put_float(buf, 36, input_voltage);
    _mav_put_float(buf, 40, cpu_load);
    _mav_put_float(buf, 44, charge_temp);
    _mav_put_float(buf, 48, ignition_angle_1);
    _mav_put_float(buf, 52, ignition_angle_2);
    _mav_put_float(buf, 56, fuel_flow);
    _mav_put_uint16_t(buf, 60, rpm_cmd);
    _mav_put_uint16_t(buf, 62, throttle_pulse);
    _mav_put_uint16_t(buf, 64, injector_duty_cycle);
    _mav_put_uint8_t(buf, 66, status);
    _mav_put_uint8_t(buf, 67, gov_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CURRAWONG_STATUS, buf, MAVLINK_MSG_ID_CURRAWONG_STATUS_MIN_LEN, MAVLINK_MSG_ID_CURRAWONG_STATUS_LEN, MAVLINK_MSG_ID_CURRAWONG_STATUS_CRC);
#else
    mavlink_currawong_status_t packet;
    packet.throttle = throttle;
    packet.rpm = rpm;
    packet.fuel_used = fuel_used;
    packet.cht = cht;
    packet.barometric_pressure = barometric_pressure;
    packet.intake_manifold_pressure = intake_manifold_pressure;
    packet.iat = iat;
    packet.fuel_press = fuel_press;
    packet.engine_time_s = engine_time_s;
    packet.input_voltage = input_voltage;
    packet.cpu_load = cpu_load;
    packet.charge_temp = charge_temp;
    packet.ignition_angle_1 = ignition_angle_1;
    packet.ignition_angle_2 = ignition_angle_2;
    packet.fuel_flow = fuel_flow;
    packet.rpm_cmd = rpm_cmd;
    packet.throttle_pulse = throttle_pulse;
    packet.injector_duty_cycle = injector_duty_cycle;
    packet.status = status;
    packet.gov_status = gov_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CURRAWONG_STATUS, (const char *)&packet, MAVLINK_MSG_ID_CURRAWONG_STATUS_MIN_LEN, MAVLINK_MSG_ID_CURRAWONG_STATUS_LEN, MAVLINK_MSG_ID_CURRAWONG_STATUS_CRC);
#endif
}

/**
 * @brief Send a currawong_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_currawong_status_send_struct(mavlink_channel_t chan, const mavlink_currawong_status_t* currawong_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_currawong_status_send(chan, currawong_status->throttle, currawong_status->rpm, currawong_status->fuel_used, currawong_status->status, currawong_status->rpm_cmd, currawong_status->throttle_pulse, currawong_status->cht, currawong_status->barometric_pressure, currawong_status->intake_manifold_pressure, currawong_status->iat, currawong_status->fuel_press, currawong_status->engine_time_s, currawong_status->input_voltage, currawong_status->gov_status, currawong_status->cpu_load, currawong_status->charge_temp, currawong_status->injector_duty_cycle, currawong_status->ignition_angle_1, currawong_status->ignition_angle_2, currawong_status->fuel_flow);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CURRAWONG_STATUS, (const char *)currawong_status, MAVLINK_MSG_ID_CURRAWONG_STATUS_MIN_LEN, MAVLINK_MSG_ID_CURRAWONG_STATUS_LEN, MAVLINK_MSG_ID_CURRAWONG_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_CURRAWONG_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_currawong_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float throttle, float rpm, float fuel_used, uint8_t status, uint16_t rpm_cmd, uint16_t throttle_pulse, float cht, float barometric_pressure, float intake_manifold_pressure, float iat, float fuel_press, float engine_time_s, float input_voltage, uint8_t gov_status, float cpu_load, float charge_temp, uint16_t injector_duty_cycle, float ignition_angle_1, float ignition_angle_2, float fuel_flow)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, throttle);
    _mav_put_float(buf, 4, rpm);
    _mav_put_float(buf, 8, fuel_used);
    _mav_put_float(buf, 12, cht);
    _mav_put_float(buf, 16, barometric_pressure);
    _mav_put_float(buf, 20, intake_manifold_pressure);
    _mav_put_float(buf, 24, iat);
    _mav_put_float(buf, 28, fuel_press);
    _mav_put_float(buf, 32, engine_time_s);
    _mav_put_float(buf, 36, input_voltage);
    _mav_put_float(buf, 40, cpu_load);
    _mav_put_float(buf, 44, charge_temp);
    _mav_put_float(buf, 48, ignition_angle_1);
    _mav_put_float(buf, 52, ignition_angle_2);
    _mav_put_float(buf, 56, fuel_flow);
    _mav_put_uint16_t(buf, 60, rpm_cmd);
    _mav_put_uint16_t(buf, 62, throttle_pulse);
    _mav_put_uint16_t(buf, 64, injector_duty_cycle);
    _mav_put_uint8_t(buf, 66, status);
    _mav_put_uint8_t(buf, 67, gov_status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CURRAWONG_STATUS, buf, MAVLINK_MSG_ID_CURRAWONG_STATUS_MIN_LEN, MAVLINK_MSG_ID_CURRAWONG_STATUS_LEN, MAVLINK_MSG_ID_CURRAWONG_STATUS_CRC);
#else
    mavlink_currawong_status_t *packet = (mavlink_currawong_status_t *)msgbuf;
    packet->throttle = throttle;
    packet->rpm = rpm;
    packet->fuel_used = fuel_used;
    packet->cht = cht;
    packet->barometric_pressure = barometric_pressure;
    packet->intake_manifold_pressure = intake_manifold_pressure;
    packet->iat = iat;
    packet->fuel_press = fuel_press;
    packet->engine_time_s = engine_time_s;
    packet->input_voltage = input_voltage;
    packet->cpu_load = cpu_load;
    packet->charge_temp = charge_temp;
    packet->ignition_angle_1 = ignition_angle_1;
    packet->ignition_angle_2 = ignition_angle_2;
    packet->fuel_flow = fuel_flow;
    packet->rpm_cmd = rpm_cmd;
    packet->throttle_pulse = throttle_pulse;
    packet->injector_duty_cycle = injector_duty_cycle;
    packet->status = status;
    packet->gov_status = gov_status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CURRAWONG_STATUS, (const char *)packet, MAVLINK_MSG_ID_CURRAWONG_STATUS_MIN_LEN, MAVLINK_MSG_ID_CURRAWONG_STATUS_LEN, MAVLINK_MSG_ID_CURRAWONG_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE CURRAWONG_STATUS UNPACKING


/**
 * @brief Get field throttle from currawong_status message
 *
 * @return [%] Throttle Position
 */
static inline float mavlink_msg_currawong_status_get_throttle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field rpm from currawong_status message
 *
 * @return [rpm] Engine Speed
 */
static inline float mavlink_msg_currawong_status_get_rpm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field fuel_used from currawong_status message
 *
 * @return [g] Fuel Consumed
 */
static inline float mavlink_msg_currawong_status_get_fuel_used(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field status from currawong_status message
 *
 * @return  ECU Status
 */
static inline uint8_t mavlink_msg_currawong_status_get_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  66);
}

/**
 * @brief Get field rpm_cmd from currawong_status message
 *
 * @return [rpm] Commanded RPM
 */
static inline uint16_t mavlink_msg_currawong_status_get_rpm_cmd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  60);
}

/**
 * @brief Get field throttle_pulse from currawong_status message
 *
 * @return [?] Throttle Pulse
 */
static inline uint16_t mavlink_msg_currawong_status_get_throttle_pulse(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  62);
}

/**
 * @brief Get field cht from currawong_status message
 *
 * @return [degC] Cylinder Head Temperature
 */
static inline float mavlink_msg_currawong_status_get_cht(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field barometric_pressure from currawong_status message
 *
 * @return [kPa] Barometric Pressure
 */
static inline float mavlink_msg_currawong_status_get_barometric_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field intake_manifold_pressure from currawong_status message
 *
 * @return [kPa] Intake Manifold Pressure
 */
static inline float mavlink_msg_currawong_status_get_intake_manifold_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field iat from currawong_status message
 *
 * @return [?] iat
 */
static inline float mavlink_msg_currawong_status_get_iat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field fuel_press from currawong_status message
 *
 * @return [Pa] Fuel Pressure
 */
static inline float mavlink_msg_currawong_status_get_fuel_press(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field engine_time_s from currawong_status message
 *
 * @return [s] Engine Run Time
 */
static inline float mavlink_msg_currawong_status_get_engine_time_s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field input_voltage from currawong_status message
 *
 * @return [V] Input Voltage
 */
static inline float mavlink_msg_currawong_status_get_input_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field gov_status from currawong_status message
 *
 * @return  Governor Status
 */
static inline uint8_t mavlink_msg_currawong_status_get_gov_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  67);
}

/**
 * @brief Get field cpu_load from currawong_status message
 *
 * @return [%] CPU Load
 */
static inline float mavlink_msg_currawong_status_get_cpu_load(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field charge_temp from currawong_status message
 *
 * @return [degC] Charge Temp
 */
static inline float mavlink_msg_currawong_status_get_charge_temp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field injector_duty_cycle from currawong_status message
 *
 * @return [?] Injection duty cycle
 */
static inline uint16_t mavlink_msg_currawong_status_get_injector_duty_cycle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  64);
}

/**
 * @brief Get field ignition_angle_1 from currawong_status message
 *
 * @return [degCrank] Ignition timing for cylinder 1 (Crank Angle degrees)
 */
static inline float mavlink_msg_currawong_status_get_ignition_angle_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field ignition_angle_2 from currawong_status message
 *
 * @return [degCrank] Ignition timing for cylinder 2 (Crank Angle degrees)
 */
static inline float mavlink_msg_currawong_status_get_ignition_angle_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field fuel_flow from currawong_status message
 *
 * @return [g/min] Fuel Flow Rate
 */
static inline float mavlink_msg_currawong_status_get_fuel_flow(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Decode a currawong_status message into a struct
 *
 * @param msg The message to decode
 * @param currawong_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_currawong_status_decode(const mavlink_message_t* msg, mavlink_currawong_status_t* currawong_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    currawong_status->throttle = mavlink_msg_currawong_status_get_throttle(msg);
    currawong_status->rpm = mavlink_msg_currawong_status_get_rpm(msg);
    currawong_status->fuel_used = mavlink_msg_currawong_status_get_fuel_used(msg);
    currawong_status->cht = mavlink_msg_currawong_status_get_cht(msg);
    currawong_status->barometric_pressure = mavlink_msg_currawong_status_get_barometric_pressure(msg);
    currawong_status->intake_manifold_pressure = mavlink_msg_currawong_status_get_intake_manifold_pressure(msg);
    currawong_status->iat = mavlink_msg_currawong_status_get_iat(msg);
    currawong_status->fuel_press = mavlink_msg_currawong_status_get_fuel_press(msg);
    currawong_status->engine_time_s = mavlink_msg_currawong_status_get_engine_time_s(msg);
    currawong_status->input_voltage = mavlink_msg_currawong_status_get_input_voltage(msg);
    currawong_status->cpu_load = mavlink_msg_currawong_status_get_cpu_load(msg);
    currawong_status->charge_temp = mavlink_msg_currawong_status_get_charge_temp(msg);
    currawong_status->ignition_angle_1 = mavlink_msg_currawong_status_get_ignition_angle_1(msg);
    currawong_status->ignition_angle_2 = mavlink_msg_currawong_status_get_ignition_angle_2(msg);
    currawong_status->fuel_flow = mavlink_msg_currawong_status_get_fuel_flow(msg);
    currawong_status->rpm_cmd = mavlink_msg_currawong_status_get_rpm_cmd(msg);
    currawong_status->throttle_pulse = mavlink_msg_currawong_status_get_throttle_pulse(msg);
    currawong_status->injector_duty_cycle = mavlink_msg_currawong_status_get_injector_duty_cycle(msg);
    currawong_status->status = mavlink_msg_currawong_status_get_status(msg);
    currawong_status->gov_status = mavlink_msg_currawong_status_get_gov_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CURRAWONG_STATUS_LEN? msg->len : MAVLINK_MSG_ID_CURRAWONG_STATUS_LEN;
        memset(currawong_status, 0, MAVLINK_MSG_ID_CURRAWONG_STATUS_LEN);
    memcpy(currawong_status, _MAV_PAYLOAD(msg), len);
#endif
}
