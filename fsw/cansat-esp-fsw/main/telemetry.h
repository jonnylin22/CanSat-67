#pragma once
#include <stdint.h>

typedef struct {
	int8_t team_id;
	int32_t mission_time_sec;
	int16_t packet_count;
	int16_t command_count;
	char mode;						// 'F' for flight and 'S' for simulation mode
	float altitude;
	float pressure;
	float temperature;
	float batt_voltage;
	float batt_current;
	uint8_t mech_state;	
	char state[32];
	char cmd_echo[64];
} Container_Telemetry_t;

typedef struct {
	int8_t team_id;
	int32_t mission_time_sec;
	int32_t gnss_time;
	int16_t packet_count;
	int16_t command_count;
	char cmd_echo[64];
	char mode;						// 'F' for flight and 'S' for simulation mode
	float altitude;
	float pressure;
	float temperature;
	float batt_voltage;
	float batt_current;
	float rot_rate_x;	
	float rot_rate_y;	
	float rot_rate_z;	
	float accel_x;
	float accel_y;
	float accel_z;
	float mag_x;
	float mag_y;
	float mag_z;
	float gps_latitude;
	float gps_longitude;
	float gps_altitude;
	uint8_t gps_sats;
	float solar_1;
	float solar_2;
	uint8_t mech_state;	
	uint32_t image_stabilization;	// HIGH LOW field? // or other data fields?
	uint32_t science_exp; 
} PQ_Telemetry_t;

void init_telemetry(Container_Telemetry_t *telemetry);
void reset_state(Container_Telemetry_t *telemetry);
void set_cmd_echo(const char *cmd, Container_Telemetry_t *telemetry);
