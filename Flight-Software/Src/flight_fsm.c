#include "../Inc/flight_fsm.h"
#include <string.h>
#include "servos.h"
#include "main.h"
#include "baro.h"
#include "gps.h"
#include "flash_memory.h"
#include <math.h>
#include <stdlib.h>

#define CONTAINER_RELEASE_ALT_PERCENTAGE 0.8
// can delete
#define PAYLOAD_RELEASE_ALT 6

// Liftoff detection variables
uint32_t launch_accel_detected_time = -1;
unsigned int negative_accel_counter = 0;

// rename to pqube
uint32_t probe_release_time = 0;
uint32_t eject_time = 0;

// change to esp idf
extern I2C_HandleTypeDef hi2c1;


uint8_t sensors_indicate_flight(Telemetry_t *telemetry){
	/*Indicates if sensors are detecting flight*/
	float alt_i = telemetry->altitude;

	for (int i = 0; i < 250; ++i){
		read_baro(&hi2c1, telemetry);
		if (fabsf(telemetry->altitude - alt_i) > POWER_RESET_MIN_ALT_CHANGE){
			return 1;
		}
		// change for esp idf
		HAL_Delay(10);
	}

	return 0;
}

void init_fsm(Telemetry_t *telemetry){
	// Get state from flash
	Telemetry_t flash_telemetry;
	if (load_flash_data(&flash_telemetry)){
		telemetry->altitude_offset = 0;
		read_baro(&hi2c1, telemetry);
		read_gps(&hi2c1, telemetry);

		// Always restore target coords
		telemetry->target_latitude = flash_telemetry.target_latitude;
		telemetry->target_longitude = flash_telemetry.target_longitude;

		if (telemetry->target_latitude == 0 || isnan(telemetry->target_latitude)){
			telemetry->target_latitude = 38.375983;
			telemetry->target_longitude = -79.607861;
		}

		// If Power-on reset (power removed and restored) and sensors indicate in flight and alt > threshold
		if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) && (telemetry->altitude + flash_telemetry.altitude_offset > MIN_RESET_ALT) && sensors_indicate_flight(telemetry) == 1){
			// reset detected
			telemetry->altitude_offset = flash_telemetry.altitude_offset;
			telemetry->alt_fused = telemetry->altitude + telemetry->altitude_offset;
			telemetry->max_altitude = flash_telemetry.max_altitude;
			telemetry->container_released = flash_telemetry.container_released;
			telemetry->paraglider_ejected = flash_telemetry.paraglider_ejected;
			telemetry->payload_released = flash_telemetry.payload_released;
			telemetry->paraglider_active = flash_telemetry.paraglider_active;
			strcpy(telemetry->state, flash_telemetry.state);
			telemetry->mission_time_hr = flash_telemetry.mission_time_hr;
			telemetry->mission_time_min = flash_telemetry.mission_time_min;
			telemetry->mission_time_sec = flash_telemetry.mission_time_sec;

			// Clear reset flags
			__HAL_RCC_CLEAR_RESET_FLAGS();
			return;
		}
	}

	// No reset detected. Reset state
	telemetry->altitude_offset = -1.0 * telemetry->altitude;
	reset_state(telemetry);

	// Clear reset flags
	__HAL_RCC_CLEAR_RESET_FLAGS();
}

void update_fsm(Telemetry_t *telemetry){
	// Eject paraglider
	if (telemetry->waiting_for_eject == 1 && telemetry->paraglider_ejected != 1 && HAL_GetTick() - probe_release_time > GLIDER_EJECTION_DELAY){
		Eject_Paraglider();
		telemetry->paraglider_ejected = 1;
		telemetry->waiting_for_eject = 0;
		telemetry->waiting_for_para_activate = 1;
		eject_time = HAL_GetTick();

//		store_flash_data(telemetry);
	}

	// Activate Paraglider
	if (telemetry->waiting_for_para_activate == 1 && telemetry->paraglider_active != 1 && HAL_GetTick() - eject_time > PARAGLIDER_ACTIVE_DELAY){
		telemetry->paraglider_active = 1;
		telemetry->waiting_for_para_activate = 0;
	}

	if (strcmp(telemetry->state, "LAUNCH_PAD") == 0){
		if (telemetry->baro_vz > 15){
			strcpy(telemetry->state, "ASCENT");
			store_flash_data(telemetry);
		}
		else if (launch_accel_detected_time == -1){
			// Acceleration not detected yet
			//changed to < for vacuum testing
			if (telemetry->accel_r > LAUNCH_ACCEL_THRESHOLD){
				// Positive acceleration detected. Begin period of waiting to get off rail.
				launch_accel_detected_time = HAL_GetTick();
				negative_accel_counter = 0;
			}
		}
		else if (HAL_GetTick() - launch_accel_detected_time > RAIL_DELAY_TIME){
			// In evaluation period. Monitor for any negative acceleration value.
			// If detected, reset the system and begin again.
			if (HAL_GetTick() - launch_accel_detected_time > RAIL_DELAY_TIME + LAUNCH_EVAL_PERIOD_TIME){
				// Enough time passed without negative acceleration. Launch detected
				strcpy(telemetry->state, "ASCENT");
				launch_accel_detected_time = -1;
				store_flash_data(telemetry);
			}
			else if (telemetry->accel_r < 9.81){
				// Negative acceleration detected. Reset system.
				if (++negative_accel_counter >= 5){
					launch_accel_detected_time = -1;
				}
			}
		}
	}
	else if (strcmp(telemetry->state, "ASCENT") == 0){
		if (telemetry->baro_vz < APOGEE_VELO_THRESHOLD){
			strcpy(telemetry->state, "APOGEE");
//			store_flash_data(telemetry);
		}
	}
	else if (strcmp(telemetry->state, "APOGEE") == 0){
		if (telemetry->sent_apogee == 1){
			strcpy(telemetry->state, "DESCENT");
			store_flash_data(telemetry);
		}
	}
	else if (strcmp(telemetry->state, "DESCENT") == 0){
//		if (telemetry->altitude <= CONTAINER_RELEASE_ALT_PERCENTAGE * telemetry->max_altitude){
		if (telemetry->alt_fused <= CONTAINER_RELEASE_ALT_PERCENTAGE * telemetry->max_altitude){
			strcpy(telemetry->state, "PROBE_RELEASE");
			Release_Container();
			telemetry->container_released = 1;
			probe_release_time = HAL_GetTick();
			telemetry->waiting_for_eject = 1;
//			store_flash_data(telemetry);
		}
	}
	else if (strcmp(telemetry->state, "PROBE_RELEASE") == 0){
		// If below release alt or detects landed (In case '0' altitude shifted during flight)
//		if (telemetry->altitude <= PAYLOAD_RELEASE_ALT || get_avg_alt_dif() > LANDED_THRESHOLD){
		if (telemetry->alt_fused <= PAYLOAD_RELEASE_ALT || (telemetry->baro_vz > LANDED_VELO_THRESHOLD && HAL_GetTick() - probe_release_time > 1500)){
			strcpy(telemetry->state, "PAYLOAD_RELEASE");
			Release_Payload();
			telemetry->payload_released = 1;
//			store_flash_data(telemetry);
		}
	}
	else if (strcmp(telemetry->state, "PAYLOAD_RELEASE") == 0){
		if (telemetry->baro_vz > LANDED_VELO_THRESHOLD && telemetry->sent_payload_release == 1){
			strcpy(telemetry->state, "LANDED");
			telemetry->paraglider_active = 0;
//			reset_flash_data();
		}
	}
}
