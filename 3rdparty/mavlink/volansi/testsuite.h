/** @file
 *    @brief MAVLink comm protocol testsuite generated from volansi.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef VOLANSI_TESTSUITE_H
#define VOLANSI_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_volansi(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_volansi(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_engine_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ENGINE_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_engine_status_t packet_in = {
        963497464,45.0,73.0,17859,17963,18067,187,254
    };
    mavlink_engine_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.fuel_used = packet_in.fuel_used;
        packet1.barometric_pressure = packet_in.barometric_pressure;
        packet1.cylinder_head_temperature = packet_in.cylinder_head_temperature;
        packet1.engine_speed = packet_in.engine_speed;
        packet1.throttle = packet_in.throttle;
        packet1.fuel_flow_rate = packet_in.fuel_flow_rate;
        packet1.status_flags = packet_in.status_flags;
        packet1.fuel_level = packet_in.fuel_level;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ENGINE_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ENGINE_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_engine_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_engine_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_engine_status_pack(system_id, component_id, &msg , packet1.status_flags , packet1.engine_speed , packet1.throttle , packet1.fuel_used , packet1.fuel_flow_rate , packet1.fuel_level , packet1.barometric_pressure , packet1.cylinder_head_temperature );
    mavlink_msg_engine_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_engine_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.status_flags , packet1.engine_speed , packet1.throttle , packet1.fuel_used , packet1.fuel_flow_rate , packet1.fuel_level , packet1.barometric_pressure , packet1.cylinder_head_temperature );
    mavlink_msg_engine_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_engine_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_engine_status_send(MAVLINK_COMM_1 , packet1.status_flags , packet1.engine_speed , packet1.throttle , packet1.fuel_used , packet1.fuel_flow_rate , packet1.fuel_level , packet1.barometric_pressure , packet1.cylinder_head_temperature );
    mavlink_msg_engine_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_skypower_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SKYPOWER_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_skypower_status_t packet_in = {
        17.0,17443,17547,17651,17755,17859,17963,18067,18171,18275,18379,18483,18587,18691,18795,18899,19003,19107,19211,19315,19419,19523,19627,19731,19835,19939,20043,20147,20251,20355,20459,20563,20667,20771,20875,20979,21083,21187,21291,21395,21499,21603,21707,21811,21915,22019,22123
    };
    mavlink_skypower_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.operating_time_s = packet_in.operating_time_s;
        packet1.engine_speed = packet_in.engine_speed;
        packet1.target_load = packet_in.target_load;
        packet1.current_load = packet_in.current_load;
        packet1.current_torque = packet_in.current_torque;
        packet1.current_power = packet_in.current_power;
        packet1.max_load = packet_in.max_load;
        packet1.rev_count = packet_in.rev_count;
        packet1.fuel_flow = packet_in.fuel_flow;
        packet1.bsfc = packet_in.bsfc;
        packet1.injection_valve_load = packet_in.injection_valve_load;
        packet1.red_factor = packet_in.red_factor;
        packet1.cyl_head_temp_1 = packet_in.cyl_head_temp_1;
        packet1.cyl_head_temp_2 = packet_in.cyl_head_temp_2;
        packet1.exhaust_temp_1 = packet_in.exhaust_temp_1;
        packet1.exhaust_temp_2 = packet_in.exhaust_temp_2;
        packet1.msl_cyl_head_temp_1 = packet_in.msl_cyl_head_temp_1;
        packet1.msl_cyl_head_temp_2 = packet_in.msl_cyl_head_temp_2;
        packet1.msl_exhaust_temp_1 = packet_in.msl_exhaust_temp_1;
        packet1.msl_exhaust_temp_2 = packet_in.msl_exhaust_temp_2;
        packet1.air_temp = packet_in.air_temp;
        packet1.ecu_temp = packet_in.ecu_temp;
        packet1.altitude = packet_in.altitude;
        packet1.isa_altitude = packet_in.isa_altitude;
        packet1.density_altitude = packet_in.density_altitude;
        packet1.pressure_altitude = packet_in.pressure_altitude;
        packet1.ambient_pressure = packet_in.ambient_pressure;
        packet1.sfc_map = packet_in.sfc_map;
        packet1.icao_fuelflow = packet_in.icao_fuelflow;
        packet1.icao_power = packet_in.icao_power;
        packet1.icao_torque = packet_in.icao_torque;
        packet1.ignition_angle = packet_in.ignition_angle;
        packet1.ignition_gap = packet_in.ignition_gap;
        packet1.throttle_angle = packet_in.throttle_angle;
        packet1.injection_angle = packet_in.injection_angle;
        packet1.injection_time = packet_in.injection_time;
        packet1.sensor_error_flags = packet_in.sensor_error_flags;
        packet1.thermal_limit_flags = packet_in.thermal_limit_flags;
        packet1.eng_supply_voltage = packet_in.eng_supply_voltage;
        packet1.gen_current_out = packet_in.gen_current_out;
        packet1.gen_phase_current = packet_in.gen_phase_current;
        packet1.gen_supply_voltage = packet_in.gen_supply_voltage;
        packet1.gen_bridge_voltage = packet_in.gen_bridge_voltage;
        packet1.gen_battery_current = packet_in.gen_battery_current;
        packet1.gen_motor_speed = packet_in.gen_motor_speed;
        packet1.gen_temp = packet_in.gen_temp;
        packet1.gen_controller_temp = packet_in.gen_controller_temp;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SKYPOWER_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SKYPOWER_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_skypower_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_skypower_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_skypower_status_pack(system_id, component_id, &msg , packet1.engine_speed , packet1.target_load , packet1.current_load , packet1.current_torque , packet1.current_power , packet1.max_load , packet1.rev_count , packet1.operating_time_s , packet1.fuel_flow , packet1.bsfc , packet1.injection_valve_load , packet1.red_factor , packet1.cyl_head_temp_1 , packet1.cyl_head_temp_2 , packet1.exhaust_temp_1 , packet1.exhaust_temp_2 , packet1.msl_cyl_head_temp_1 , packet1.msl_cyl_head_temp_2 , packet1.msl_exhaust_temp_1 , packet1.msl_exhaust_temp_2 , packet1.air_temp , packet1.ecu_temp , packet1.altitude , packet1.isa_altitude , packet1.density_altitude , packet1.pressure_altitude , packet1.ambient_pressure , packet1.sfc_map , packet1.icao_fuelflow , packet1.icao_power , packet1.icao_torque , packet1.ignition_angle , packet1.ignition_gap , packet1.throttle_angle , packet1.injection_angle , packet1.injection_time , packet1.sensor_error_flags , packet1.thermal_limit_flags , packet1.eng_supply_voltage , packet1.gen_current_out , packet1.gen_phase_current , packet1.gen_supply_voltage , packet1.gen_bridge_voltage , packet1.gen_battery_current , packet1.gen_motor_speed , packet1.gen_temp , packet1.gen_controller_temp );
    mavlink_msg_skypower_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_skypower_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.engine_speed , packet1.target_load , packet1.current_load , packet1.current_torque , packet1.current_power , packet1.max_load , packet1.rev_count , packet1.operating_time_s , packet1.fuel_flow , packet1.bsfc , packet1.injection_valve_load , packet1.red_factor , packet1.cyl_head_temp_1 , packet1.cyl_head_temp_2 , packet1.exhaust_temp_1 , packet1.exhaust_temp_2 , packet1.msl_cyl_head_temp_1 , packet1.msl_cyl_head_temp_2 , packet1.msl_exhaust_temp_1 , packet1.msl_exhaust_temp_2 , packet1.air_temp , packet1.ecu_temp , packet1.altitude , packet1.isa_altitude , packet1.density_altitude , packet1.pressure_altitude , packet1.ambient_pressure , packet1.sfc_map , packet1.icao_fuelflow , packet1.icao_power , packet1.icao_torque , packet1.ignition_angle , packet1.ignition_gap , packet1.throttle_angle , packet1.injection_angle , packet1.injection_time , packet1.sensor_error_flags , packet1.thermal_limit_flags , packet1.eng_supply_voltage , packet1.gen_current_out , packet1.gen_phase_current , packet1.gen_supply_voltage , packet1.gen_bridge_voltage , packet1.gen_battery_current , packet1.gen_motor_speed , packet1.gen_temp , packet1.gen_controller_temp );
    mavlink_msg_skypower_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_skypower_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_skypower_status_send(MAVLINK_COMM_1 , packet1.engine_speed , packet1.target_load , packet1.current_load , packet1.current_torque , packet1.current_power , packet1.max_load , packet1.rev_count , packet1.operating_time_s , packet1.fuel_flow , packet1.bsfc , packet1.injection_valve_load , packet1.red_factor , packet1.cyl_head_temp_1 , packet1.cyl_head_temp_2 , packet1.exhaust_temp_1 , packet1.exhaust_temp_2 , packet1.msl_cyl_head_temp_1 , packet1.msl_cyl_head_temp_2 , packet1.msl_exhaust_temp_1 , packet1.msl_exhaust_temp_2 , packet1.air_temp , packet1.ecu_temp , packet1.altitude , packet1.isa_altitude , packet1.density_altitude , packet1.pressure_altitude , packet1.ambient_pressure , packet1.sfc_map , packet1.icao_fuelflow , packet1.icao_power , packet1.icao_torque , packet1.ignition_angle , packet1.ignition_gap , packet1.throttle_angle , packet1.injection_angle , packet1.injection_time , packet1.sensor_error_flags , packet1.thermal_limit_flags , packet1.eng_supply_voltage , packet1.gen_current_out , packet1.gen_phase_current , packet1.gen_supply_voltage , packet1.gen_bridge_voltage , packet1.gen_battery_current , packet1.gen_motor_speed , packet1.gen_temp , packet1.gen_controller_temp );
    mavlink_msg_skypower_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_currawong_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CURRAWONG_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_currawong_status_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0,297.0,325.0,353.0,381.0,409.0,20355,20459,20563,75,142
    };
    mavlink_currawong_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.throttle = packet_in.throttle;
        packet1.rpm = packet_in.rpm;
        packet1.fuel_used = packet_in.fuel_used;
        packet1.cht = packet_in.cht;
        packet1.barometric_pressure = packet_in.barometric_pressure;
        packet1.intake_manifold_pressure = packet_in.intake_manifold_pressure;
        packet1.iat = packet_in.iat;
        packet1.fuel_press = packet_in.fuel_press;
        packet1.engine_time_s = packet_in.engine_time_s;
        packet1.input_voltage = packet_in.input_voltage;
        packet1.cpu_load = packet_in.cpu_load;
        packet1.charge_temp = packet_in.charge_temp;
        packet1.ignition_angle_1 = packet_in.ignition_angle_1;
        packet1.ignition_angle_2 = packet_in.ignition_angle_2;
        packet1.fuel_flow = packet_in.fuel_flow;
        packet1.rpm_cmd = packet_in.rpm_cmd;
        packet1.throttle_pulse = packet_in.throttle_pulse;
        packet1.injector_duty_cycle = packet_in.injector_duty_cycle;
        packet1.status = packet_in.status;
        packet1.gov_status = packet_in.gov_status;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CURRAWONG_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CURRAWONG_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_currawong_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_currawong_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_currawong_status_pack(system_id, component_id, &msg , packet1.throttle , packet1.rpm , packet1.fuel_used , packet1.status , packet1.rpm_cmd , packet1.throttle_pulse , packet1.cht , packet1.barometric_pressure , packet1.intake_manifold_pressure , packet1.iat , packet1.fuel_press , packet1.engine_time_s , packet1.input_voltage , packet1.gov_status , packet1.cpu_load , packet1.charge_temp , packet1.injector_duty_cycle , packet1.ignition_angle_1 , packet1.ignition_angle_2 , packet1.fuel_flow );
    mavlink_msg_currawong_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_currawong_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.throttle , packet1.rpm , packet1.fuel_used , packet1.status , packet1.rpm_cmd , packet1.throttle_pulse , packet1.cht , packet1.barometric_pressure , packet1.intake_manifold_pressure , packet1.iat , packet1.fuel_press , packet1.engine_time_s , packet1.input_voltage , packet1.gov_status , packet1.cpu_load , packet1.charge_temp , packet1.injector_duty_cycle , packet1.ignition_angle_1 , packet1.ignition_angle_2 , packet1.fuel_flow );
    mavlink_msg_currawong_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_currawong_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_currawong_status_send(MAVLINK_COMM_1 , packet1.throttle , packet1.rpm , packet1.fuel_used , packet1.status , packet1.rpm_cmd , packet1.throttle_pulse , packet1.cht , packet1.barometric_pressure , packet1.intake_manifold_pressure , packet1.iat , packet1.fuel_press , packet1.engine_time_s , packet1.input_voltage , packet1.gov_status , packet1.cpu_load , packet1.charge_temp , packet1.injector_duty_cycle , packet1.ignition_angle_1 , packet1.ignition_angle_2 , packet1.fuel_flow );
    mavlink_msg_currawong_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_volansi(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_engine_status(system_id, component_id, last_msg);
    mavlink_test_skypower_status(system_id, component_id, last_msg);
    mavlink_test_currawong_status(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // VOLANSI_TESTSUITE_H
