#pragma once
// MESSAGE ENGINE_STATUS PACKING

#define MAVLINK_MSG_ID_ENGINE_STATUS 225


typedef struct __mavlink_engine_status_t {
 uint32_t fuel_used; /*< [mL] Fuel Consumed (mL)*/
 float barometric_pressure; /*< [kPa] Barometric Pressure (kPa)*/
 float cylinder_head_temperature; /*< [degC] cylinder_head_temperature (degC)*/
 uint16_t engine_speed; /*< [rpm] RPM*/
 uint16_t throttle; /*< [%] Throttle Setpoint (%)*/
 uint16_t fuel_flow_rate; /*< [mL/hr] Fuel Flow Rate (mL/hour)*/
 uint8_t status_flags; /*<  Engine status flags*/
 int8_t fuel_level; /*< [%] Fuel Tank Level (%)*/
} mavlink_engine_status_t;

#define MAVLINK_MSG_ID_ENGINE_STATUS_LEN 20
#define MAVLINK_MSG_ID_ENGINE_STATUS_MIN_LEN 20
#define MAVLINK_MSG_ID_225_LEN 20
#define MAVLINK_MSG_ID_225_MIN_LEN 20

#define MAVLINK_MSG_ID_ENGINE_STATUS_CRC 10
#define MAVLINK_MSG_ID_225_CRC 10



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ENGINE_STATUS { \
    225, \
    "ENGINE_STATUS", \
    8, \
    {  { "status_flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_engine_status_t, status_flags) }, \
         { "engine_speed", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_engine_status_t, engine_speed) }, \
         { "throttle", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_engine_status_t, throttle) }, \
         { "fuel_used", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_engine_status_t, fuel_used) }, \
         { "fuel_flow_rate", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_engine_status_t, fuel_flow_rate) }, \
         { "fuel_level", NULL, MAVLINK_TYPE_INT8_T, 0, 19, offsetof(mavlink_engine_status_t, fuel_level) }, \
         { "barometric_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_engine_status_t, barometric_pressure) }, \
         { "cylinder_head_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_engine_status_t, cylinder_head_temperature) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ENGINE_STATUS { \
    "ENGINE_STATUS", \
    8, \
    {  { "status_flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_engine_status_t, status_flags) }, \
         { "engine_speed", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_engine_status_t, engine_speed) }, \
         { "throttle", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_engine_status_t, throttle) }, \
         { "fuel_used", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_engine_status_t, fuel_used) }, \
         { "fuel_flow_rate", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_engine_status_t, fuel_flow_rate) }, \
         { "fuel_level", NULL, MAVLINK_TYPE_INT8_T, 0, 19, offsetof(mavlink_engine_status_t, fuel_level) }, \
         { "barometric_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_engine_status_t, barometric_pressure) }, \
         { "cylinder_head_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_engine_status_t, cylinder_head_temperature) }, \
         } \
}
#endif

/**
 * @brief Pack a engine_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param status_flags  Engine status flags
 * @param engine_speed [rpm] RPM
 * @param throttle [%] Throttle Setpoint (%)
 * @param fuel_used [mL] Fuel Consumed (mL)
 * @param fuel_flow_rate [mL/hr] Fuel Flow Rate (mL/hour)
 * @param fuel_level [%] Fuel Tank Level (%)
 * @param barometric_pressure [kPa] Barometric Pressure (kPa)
 * @param cylinder_head_temperature [degC] cylinder_head_temperature (degC)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_engine_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t status_flags, uint16_t engine_speed, uint16_t throttle, uint32_t fuel_used, uint16_t fuel_flow_rate, int8_t fuel_level, float barometric_pressure, float cylinder_head_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ENGINE_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, fuel_used);
    _mav_put_float(buf, 4, barometric_pressure);
    _mav_put_float(buf, 8, cylinder_head_temperature);
    _mav_put_uint16_t(buf, 12, engine_speed);
    _mav_put_uint16_t(buf, 14, throttle);
    _mav_put_uint16_t(buf, 16, fuel_flow_rate);
    _mav_put_uint8_t(buf, 18, status_flags);
    _mav_put_int8_t(buf, 19, fuel_level);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ENGINE_STATUS_LEN);
#else
    mavlink_engine_status_t packet;
    packet.fuel_used = fuel_used;
    packet.barometric_pressure = barometric_pressure;
    packet.cylinder_head_temperature = cylinder_head_temperature;
    packet.engine_speed = engine_speed;
    packet.throttle = throttle;
    packet.fuel_flow_rate = fuel_flow_rate;
    packet.status_flags = status_flags;
    packet.fuel_level = fuel_level;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ENGINE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ENGINE_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ENGINE_STATUS_MIN_LEN, MAVLINK_MSG_ID_ENGINE_STATUS_LEN, MAVLINK_MSG_ID_ENGINE_STATUS_CRC);
}

/**
 * @brief Pack a engine_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param status_flags  Engine status flags
 * @param engine_speed [rpm] RPM
 * @param throttle [%] Throttle Setpoint (%)
 * @param fuel_used [mL] Fuel Consumed (mL)
 * @param fuel_flow_rate [mL/hr] Fuel Flow Rate (mL/hour)
 * @param fuel_level [%] Fuel Tank Level (%)
 * @param barometric_pressure [kPa] Barometric Pressure (kPa)
 * @param cylinder_head_temperature [degC] cylinder_head_temperature (degC)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_engine_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t status_flags,uint16_t engine_speed,uint16_t throttle,uint32_t fuel_used,uint16_t fuel_flow_rate,int8_t fuel_level,float barometric_pressure,float cylinder_head_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ENGINE_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, fuel_used);
    _mav_put_float(buf, 4, barometric_pressure);
    _mav_put_float(buf, 8, cylinder_head_temperature);
    _mav_put_uint16_t(buf, 12, engine_speed);
    _mav_put_uint16_t(buf, 14, throttle);
    _mav_put_uint16_t(buf, 16, fuel_flow_rate);
    _mav_put_uint8_t(buf, 18, status_flags);
    _mav_put_int8_t(buf, 19, fuel_level);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ENGINE_STATUS_LEN);
#else
    mavlink_engine_status_t packet;
    packet.fuel_used = fuel_used;
    packet.barometric_pressure = barometric_pressure;
    packet.cylinder_head_temperature = cylinder_head_temperature;
    packet.engine_speed = engine_speed;
    packet.throttle = throttle;
    packet.fuel_flow_rate = fuel_flow_rate;
    packet.status_flags = status_flags;
    packet.fuel_level = fuel_level;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ENGINE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ENGINE_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ENGINE_STATUS_MIN_LEN, MAVLINK_MSG_ID_ENGINE_STATUS_LEN, MAVLINK_MSG_ID_ENGINE_STATUS_CRC);
}

/**
 * @brief Encode a engine_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param engine_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_engine_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_engine_status_t* engine_status)
{
    return mavlink_msg_engine_status_pack(system_id, component_id, msg, engine_status->status_flags, engine_status->engine_speed, engine_status->throttle, engine_status->fuel_used, engine_status->fuel_flow_rate, engine_status->fuel_level, engine_status->barometric_pressure, engine_status->cylinder_head_temperature);
}

/**
 * @brief Encode a engine_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param engine_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_engine_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_engine_status_t* engine_status)
{
    return mavlink_msg_engine_status_pack_chan(system_id, component_id, chan, msg, engine_status->status_flags, engine_status->engine_speed, engine_status->throttle, engine_status->fuel_used, engine_status->fuel_flow_rate, engine_status->fuel_level, engine_status->barometric_pressure, engine_status->cylinder_head_temperature);
}

/**
 * @brief Send a engine_status message
 * @param chan MAVLink channel to send the message
 *
 * @param status_flags  Engine status flags
 * @param engine_speed [rpm] RPM
 * @param throttle [%] Throttle Setpoint (%)
 * @param fuel_used [mL] Fuel Consumed (mL)
 * @param fuel_flow_rate [mL/hr] Fuel Flow Rate (mL/hour)
 * @param fuel_level [%] Fuel Tank Level (%)
 * @param barometric_pressure [kPa] Barometric Pressure (kPa)
 * @param cylinder_head_temperature [degC] cylinder_head_temperature (degC)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_engine_status_send(mavlink_channel_t chan, uint8_t status_flags, uint16_t engine_speed, uint16_t throttle, uint32_t fuel_used, uint16_t fuel_flow_rate, int8_t fuel_level, float barometric_pressure, float cylinder_head_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ENGINE_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, fuel_used);
    _mav_put_float(buf, 4, barometric_pressure);
    _mav_put_float(buf, 8, cylinder_head_temperature);
    _mav_put_uint16_t(buf, 12, engine_speed);
    _mav_put_uint16_t(buf, 14, throttle);
    _mav_put_uint16_t(buf, 16, fuel_flow_rate);
    _mav_put_uint8_t(buf, 18, status_flags);
    _mav_put_int8_t(buf, 19, fuel_level);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ENGINE_STATUS, buf, MAVLINK_MSG_ID_ENGINE_STATUS_MIN_LEN, MAVLINK_MSG_ID_ENGINE_STATUS_LEN, MAVLINK_MSG_ID_ENGINE_STATUS_CRC);
#else
    mavlink_engine_status_t packet;
    packet.fuel_used = fuel_used;
    packet.barometric_pressure = barometric_pressure;
    packet.cylinder_head_temperature = cylinder_head_temperature;
    packet.engine_speed = engine_speed;
    packet.throttle = throttle;
    packet.fuel_flow_rate = fuel_flow_rate;
    packet.status_flags = status_flags;
    packet.fuel_level = fuel_level;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ENGINE_STATUS, (const char *)&packet, MAVLINK_MSG_ID_ENGINE_STATUS_MIN_LEN, MAVLINK_MSG_ID_ENGINE_STATUS_LEN, MAVLINK_MSG_ID_ENGINE_STATUS_CRC);
#endif
}

/**
 * @brief Send a engine_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_engine_status_send_struct(mavlink_channel_t chan, const mavlink_engine_status_t* engine_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_engine_status_send(chan, engine_status->status_flags, engine_status->engine_speed, engine_status->throttle, engine_status->fuel_used, engine_status->fuel_flow_rate, engine_status->fuel_level, engine_status->barometric_pressure, engine_status->cylinder_head_temperature);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ENGINE_STATUS, (const char *)engine_status, MAVLINK_MSG_ID_ENGINE_STATUS_MIN_LEN, MAVLINK_MSG_ID_ENGINE_STATUS_LEN, MAVLINK_MSG_ID_ENGINE_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ENGINE_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_engine_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t status_flags, uint16_t engine_speed, uint16_t throttle, uint32_t fuel_used, uint16_t fuel_flow_rate, int8_t fuel_level, float barometric_pressure, float cylinder_head_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, fuel_used);
    _mav_put_float(buf, 4, barometric_pressure);
    _mav_put_float(buf, 8, cylinder_head_temperature);
    _mav_put_uint16_t(buf, 12, engine_speed);
    _mav_put_uint16_t(buf, 14, throttle);
    _mav_put_uint16_t(buf, 16, fuel_flow_rate);
    _mav_put_uint8_t(buf, 18, status_flags);
    _mav_put_int8_t(buf, 19, fuel_level);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ENGINE_STATUS, buf, MAVLINK_MSG_ID_ENGINE_STATUS_MIN_LEN, MAVLINK_MSG_ID_ENGINE_STATUS_LEN, MAVLINK_MSG_ID_ENGINE_STATUS_CRC);
#else
    mavlink_engine_status_t *packet = (mavlink_engine_status_t *)msgbuf;
    packet->fuel_used = fuel_used;
    packet->barometric_pressure = barometric_pressure;
    packet->cylinder_head_temperature = cylinder_head_temperature;
    packet->engine_speed = engine_speed;
    packet->throttle = throttle;
    packet->fuel_flow_rate = fuel_flow_rate;
    packet->status_flags = status_flags;
    packet->fuel_level = fuel_level;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ENGINE_STATUS, (const char *)packet, MAVLINK_MSG_ID_ENGINE_STATUS_MIN_LEN, MAVLINK_MSG_ID_ENGINE_STATUS_LEN, MAVLINK_MSG_ID_ENGINE_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE ENGINE_STATUS UNPACKING


/**
 * @brief Get field status_flags from engine_status message
 *
 * @return  Engine status flags
 */
static inline uint8_t mavlink_msg_engine_status_get_status_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field engine_speed from engine_status message
 *
 * @return [rpm] RPM
 */
static inline uint16_t mavlink_msg_engine_status_get_engine_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field throttle from engine_status message
 *
 * @return [%] Throttle Setpoint (%)
 */
static inline uint16_t mavlink_msg_engine_status_get_throttle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  14);
}

/**
 * @brief Get field fuel_used from engine_status message
 *
 * @return [mL] Fuel Consumed (mL)
 */
static inline uint32_t mavlink_msg_engine_status_get_fuel_used(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field fuel_flow_rate from engine_status message
 *
 * @return [mL/hr] Fuel Flow Rate (mL/hour)
 */
static inline uint16_t mavlink_msg_engine_status_get_fuel_flow_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field fuel_level from engine_status message
 *
 * @return [%] Fuel Tank Level (%)
 */
static inline int8_t mavlink_msg_engine_status_get_fuel_level(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  19);
}

/**
 * @brief Get field barometric_pressure from engine_status message
 *
 * @return [kPa] Barometric Pressure (kPa)
 */
static inline float mavlink_msg_engine_status_get_barometric_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field cylinder_head_temperature from engine_status message
 *
 * @return [degC] cylinder_head_temperature (degC)
 */
static inline float mavlink_msg_engine_status_get_cylinder_head_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a engine_status message into a struct
 *
 * @param msg The message to decode
 * @param engine_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_engine_status_decode(const mavlink_message_t* msg, mavlink_engine_status_t* engine_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    engine_status->fuel_used = mavlink_msg_engine_status_get_fuel_used(msg);
    engine_status->barometric_pressure = mavlink_msg_engine_status_get_barometric_pressure(msg);
    engine_status->cylinder_head_temperature = mavlink_msg_engine_status_get_cylinder_head_temperature(msg);
    engine_status->engine_speed = mavlink_msg_engine_status_get_engine_speed(msg);
    engine_status->throttle = mavlink_msg_engine_status_get_throttle(msg);
    engine_status->fuel_flow_rate = mavlink_msg_engine_status_get_fuel_flow_rate(msg);
    engine_status->status_flags = mavlink_msg_engine_status_get_status_flags(msg);
    engine_status->fuel_level = mavlink_msg_engine_status_get_fuel_level(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ENGINE_STATUS_LEN? msg->len : MAVLINK_MSG_ID_ENGINE_STATUS_LEN;
        memset(engine_status, 0, MAVLINK_MSG_ID_ENGINE_STATUS_LEN);
    memcpy(engine_status, _MAV_PAYLOAD(msg), len);
#endif
}
