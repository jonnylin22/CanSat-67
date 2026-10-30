#include "telemetry.h"
#include "flash_memory.h"
#include <string.h>

void init_telemetry(Telemetry_t *telemetry){
	// Initialize telemtry values if they should not start as 0
	telemetry->mode = 'F';
	strcpy(telemetry->state, "LAUNCH_PAD");
	telemetry->telemetry_status = 1;
	strcpy(telemetry->cmd_echo, "N/A");
}

void reset_container_state(Container_Telemetry_t *container_telemetry){
	strcpy(telemetry->state, "LAUNCH_PAD");
	container_telemetry->mission_time_sec = 0;
	container_telemetry->packet_count = 0;
	container_telemetry->command_count = 0;
	container_telemetry->mode = 0;
	container_telemetry->altitude = 0;
	container_telemetry->pressure = 0;
	container_telemetry->temperature = 0;


	float altitude;
	float pressure;
	float temperature;
	float batt_voltage;
	float batt_current;
	uint8_t mech_state;	
	char state[32];
	char cmd_echo[64];


	Reset_Payload();
	Reset_Container();

	store_flash_data(telemetry);
}

void reset_pq(PQ_Telemetry_t *pq_telemetry) {

}

void set_cmd_echo(const char *cmd, Telemetry_t *telemetry)
{
	memset(telemetry->cmd_echo, '\0', sizeof(telemetry->cmd_echo));
	strncpy(telemetry->cmd_echo, cmd, strlen(cmd));
}
