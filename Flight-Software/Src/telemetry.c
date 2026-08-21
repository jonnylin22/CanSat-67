#include "telemetry.h"
#include "flash_memory.h"
#include "servos.h"
#include <string.h>

void init_telemetry(Telemetry_t *telemetry){
	// Initialize telemtry values if they should not start as 0
	telemetry->mode = 'F';
	strcpy(telemetry->state, "LAUNCH_PAD");
	telemetry->telemetry_status = 1;
	strcpy(telemetry->cmd_echo, "N/A");
}

void reset_state(Telemetry_t *telemetry){
	strcpy(telemetry->state, "LAUNCH_PAD");
	telemetry->max_altitude = 0;
	telemetry->sent_apogee = 0;
	telemetry->sent_payload_release = 0;
	telemetry->container_released = 0;
	// remove paraglider_ejected and active and waiting to eject and waiting for para_act
	telemetry->paraglider_ejected = 0;
	telemetry->payload_released = 0;
	telemetry->paraglider_active = 0;
	telemetry->packet_count = 0;
	telemetry->alt_fused = 0;
	telemetry->waiting_for_eject = 0;
	telemetry->waiting_for_para_activate = 0;

	Reset_Payload();
	Reset_Container();

	store_flash_data(telemetry);
}

void set_cmd_echo(const char *cmd, Telemetry_t *telemetry)
{
	memset(telemetry->cmd_echo, '\0', sizeof(telemetry->cmd_echo));
	strncpy(telemetry->cmd_echo, cmd, strlen(cmd));
}
