#include "flash_memory.h"
#include <string.h>
#include <math.h>

extern float heading_when_north;

uint32_t time_seconds(uint8_t hr, uint8_t min, uint8_t sec){
	return 3600 * hr + 60 * min + sec;
}

// Time difference from mission time to gps time
int32_t get_time_dif(Telemetry_t *telemetry){
	return time_seconds(telemetry->mission_time_hr, telemetry->mission_time_min, telemetry->mission_time_sec) -
			time_seconds(telemetry->gps_time_hr, telemetry->gps_time_min, telemetry->gps_time_sec);
}

void set_mission_time(Telemetry_t *telemetry, uint32_t time_dif){
	int32_t mission_time = (time_seconds(telemetry->gps_time_hr, telemetry->gps_time_min, telemetry->gps_time_sec) + time_dif) % 86400;
	if (mission_time < 0) mission_time += 86400;
	telemetry->mission_time_sec = mission_time % 60;
	mission_time -= telemetry->mission_time_sec;
	telemetry->mission_time_min = (mission_time % 3600) / 60;
	mission_time -= telemetry->mission_time_min;
	telemetry->mission_time_hr = mission_time / 3600;
}

uint32_t state_number(Telemetry_t *telemetry){
	if (strcmp(telemetry->state, "LAUNCH_PAD") == 0){
		return 0;
	}
	else if (strcmp(telemetry->state, "ASCENT") == 0){
		return 1;
	}
	else if (strcmp(telemetry->state, "APOGEE") == 0){
		return 2;
	}
	else if (strcmp(telemetry->state, "DESCENT") == 0){
		return 3;
	}
	else if (strcmp(telemetry->state, "PROBE_RELEASE") == 0){
		return 4;
	}
	else if (strcmp(telemetry->state, "PAYLOAD_RELEASE") == 0){
		return 5;
	}
	else if (strcmp(telemetry->state, "LANDED") == 0){
		return 6;
	}
	return 0;
}

void set_state_str(Telemetry_t *telemetry, uint32_t state){
	switch(state){
	case 1:
		strcpy(telemetry->state, "ASCENT");
		break;
	case 2:
		strcpy(telemetry->state, "APOGEE");
		break;
	case 3:
		strcpy(telemetry->state, "DESCENT");
		break;
	case 4:
		strcpy(telemetry->state, "PROBE_RELEASE");
		break;
	case 5:
		strcpy(telemetry->state, "PAYLOAD_RELEASE");
		break;
	case 6:
		strcpy(telemetry->state, "LANDED");
		break;
	default:
		strcpy(telemetry->state, "LAUNCH_PAD");
	}
}

void store_flash_data(Telemetry_t *telemetry){
	// Store altitude offset, magnetic offsets, mission time
	HAL_FLASH_Unlock();

	FLASH_Erase_Sector(SECTOR, FLASH_VOLTAGE_RANGE_2);
//	HAL_Delay(100);

	uint32_t altitude_offset_bits = 0;
	uint32_t apogee_altitude_bits, target_lat_bits, target_long_bits, heading_when_north_bits;
	uint32_t container_released_bits = telemetry->container_released ? 1 : 0;
	uint32_t glider_ejected_bits = telemetry->paraglider_ejected ? 1 : 0;
	uint32_t payload_released_bits = telemetry->payload_released ? 1 : 0;
	uint32_t glider_active_bits = telemetry->paraglider_active ? 1 : 0;

	// Copy the float data into the 32-bit unsigned integer variables
	if (!isnan(telemetry->altitude_offset)){
		memcpy(&altitude_offset_bits, &(telemetry->altitude_offset), sizeof(telemetry->altitude_offset));
	}
	memcpy(&apogee_altitude_bits, &(telemetry->max_altitude), sizeof(telemetry->max_altitude));
	memcpy(&target_lat_bits, &(telemetry->target_latitude), sizeof(telemetry->target_latitude));
	memcpy(&target_long_bits, &(telemetry->target_longitude), sizeof(telemetry->target_longitude));
	memcpy(&heading_when_north_bits, &heading_when_north, sizeof(heading_when_north));

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_ALTITUDE_OFFSET_ADDRESS, altitude_offset_bits);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_TIME_DIF_ADDRESS, get_time_dif(telemetry));
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_APOGEE_ALT_ADDRESS, apogee_altitude_bits);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STATE_ADDRESS, state_number(telemetry));
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_CONTAINER_RELEASED_ADDRESS, container_released_bits);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_GLIDER_EJECTED_ADDRESS, glider_ejected_bits);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_PAYLOAD_RELEASED_ADDRESS, payload_released_bits);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_GLIDER_ACTIVE_ADDRESS, glider_active_bits);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_TARGET_LAT_ADDRESS, target_lat_bits);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_TARGET_LONG_ADDRESS, target_long_bits);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_HEADING_WHEN_NORTH_ADDRESS, heading_when_north_bits);

//	HAL_Delay(100);

	HAL_FLASH_Lock();
}

uint8_t load_flash_data(Telemetry_t *telemetry){
//	HAL_FLASH_Unlock();

	// Check if uninitialized
	uint32_t raw = *(uint32_t*)FLASH_STATE_ADDRESS;
	if (raw == 0xFFFFFFFF){
		// Flash uninitialized
		return 0;
	}

	memcpy(&(telemetry->altitude_offset), (float*)FLASH_ALTITUDE_OFFSET_ADDRESS, sizeof(float));
	uint32_t time_dif = *(uint32_t*)FLASH_TIME_DIF_ADDRESS;
	set_mission_time(telemetry, time_dif);
	memcpy(&(telemetry->max_altitude), (float*)FLASH_APOGEE_ALT_ADDRESS, sizeof(float));
	telemetry->container_released = (*(uint32_t*)FLASH_CONTAINER_RELEASED_ADDRESS) ? 1 : 0;
	telemetry->paraglider_ejected = (*(uint32_t*)FLASH_GLIDER_EJECTED_ADDRESS) ? 1 : 0;
	telemetry->payload_released = (*(uint32_t*)FLASH_PAYLOAD_RELEASED_ADDRESS) ? 1 : 0;
	telemetry->paraglider_active = (*(uint32_t*)FLASH_GLIDER_ACTIVE_ADDRESS) ? 1 : 0;
	memcpy(&(telemetry->target_latitude), (float*)FLASH_TARGET_LAT_ADDRESS, sizeof(float));
	memcpy(&(telemetry->target_longitude), (float*)FLASH_TARGET_LONG_ADDRESS, sizeof(float));
	memcpy(&heading_when_north, (float*)FLASH_HEADING_WHEN_NORTH_ADDRESS, sizeof(float));

	set_state_str(telemetry, *(uint32_t*)FLASH_STATE_ADDRESS);

//	HAL_FLASH_Lock();

	return 1;
}

void reset_flash_data(){
	HAL_FLASH_Unlock();
	FLASH_Erase_Sector(SECTOR, FLASH_VOLTAGE_RANGE_2);
	HAL_FLASH_Lock();
}
