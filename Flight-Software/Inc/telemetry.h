#pragma once
#include <stdint.h>


// update with new fields from Jonny
typedef struct {
	int8_t mission_time_hr;
	int8_t mission_time_min;
	int8_t mission_time_sec;
	int16_t packet_count;
	char mode;
	char state[14];
	float altitude;
	float alt_fused;
	float temperature;
	float pressure;
	float voltage;
	float current;
	float gyro_r;
	float gyro_p;
	float gyro_y;
	float accel_r;
	float accel_p;
	float accel_y;
	uint8_t gps_time_hr;
	uint8_t gps_time_min;
	uint8_t gps_time_sec;
	float gps_altitude;
	float gps_latitude;
	float gps_longitude;
	float heading;
	uint8_t gps_sats;
	char cmd_echo[64];
	uint8_t container_released;
	uint8_t payload_released;
	uint8_t paraglider_active;
	uint8_t paraglider_ejected;
	uint8_t waiting_for_eject;
	uint8_t waiting_for_para_activate;
	float target_latitude;
	float target_longitude;
	uint8_t sim_enabled;
	uint8_t telemetry_status;
	float altitude_offset;
	float max_altitude;
	uint8_t sent_apogee; // Helps determine when to switch out of apogee state
	uint8_t sent_payload_release; // Ensure that payload release state is sent because it happens close to landed state
	float q0;
	float q1;
	float q2;
	float q3;
	float accel_world_x;
	float accel_world_y;
	float accel_world_z;
	float velocity_world_z;
	float baro_vz;
	float tilt;
} Telemetry_t;

void init_telemetry(Telemetry_t *telemetry);
void reset_state(Telemetry_t *telemetry);
void set_cmd_echo(const char *cmd, Telemetry_t *telemetry);
