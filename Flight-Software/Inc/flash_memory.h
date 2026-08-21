#pragma once
#include "telemetry.h"

// Flash Memory
// delete and rewrite with non-volatile storage
#define SECTOR 11
#define FLASH_ALTITUDE_OFFSET_ADDRESS 0x080E0000
#define FLASH_TIME_DIF_ADDRESS 0x080E0004
#define FLASH_APOGEE_ALT_ADDRESS 0x080E0008
#define FLASH_STATE_ADDRESS 0x080E000C
#define FLASH_CONTAINER_RELEASED_ADDRESS 0x080E0010
#define FLASH_GLIDER_EJECTED_ADDRESS 0x080E0014
#define FLASH_PAYLOAD_RELEASED_ADDRESS 0x080E0018
#define FLASH_GLIDER_ACTIVE_ADDRESS 0x080E001C
#define FLASH_TARGET_LAT_ADDRESS 0x080E0020
#define FLASH_TARGET_LONG_ADDRESS 0x080E0024
#define FLASH_HEADING_WHEN_NORTH_ADDRESS 0x080E0028

void store_flash_data(Telemetry_t *telemetry);
uint8_t load_flash_data(Telemetry_t *telemetry);
void reset_flash_data();
