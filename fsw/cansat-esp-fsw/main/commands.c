#include "main.h"
#include "commands.h"
#include "servos.h"
#include "baro.h"
#include "gps.h"
#include "flight_fsm.h"
#include "xbee.h"
#include "flash_memory.h"
#include "paraglider.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

char command[RX_BFR_SIZE-1];
extern I2C_HandleTypeDef hi2c1;
extern Telemetry_t telemetry;

// Commands
char sim_command[14];
char simp_command[15];
char set_time_command[13];
char cal_alt_command[14];
char tel_on_command[15];
char tel_off_command[16];
char release_payload_command[25];
char reset_release_payload_command[26];
char release_container_command[27];
char reset_release_container_command[28];
char glider_on_command[24];
char glider_off_command[25];
char eject_command[20];
char reset_state_command[14];
char set_coords_command[14];
char set_north_command[15];

void init_commands(void)
{
	snprintf(sim_command, sizeof(sim_command), "CMD,%s,SIM,", TEAM_ID);
	snprintf(simp_command, sizeof(simp_command), "CMD,%s,SIMP,", TEAM_ID);
	snprintf(set_time_command, sizeof(set_time_command), "CMD,%s,ST,", TEAM_ID);
	snprintf(cal_alt_command, sizeof(cal_alt_command), "CMD,%s,CAL", TEAM_ID);
	snprintf(tel_on_command, sizeof(tel_on_command), "CMD,%s,CX,ON", TEAM_ID);\
	snprintf(tel_off_command, sizeof(tel_off_command), "CMD,%s,CX,OFF", TEAM_ID);
	snprintf(release_payload_command, sizeof(release_payload_command), "CMD,%s,MEC,PAYLOAD,ON", TEAM_ID);
	snprintf(reset_release_payload_command, sizeof(reset_release_payload_command), "CMD,%s,MEC,PAYLOAD,OFF", TEAM_ID);
	snprintf(release_container_command, sizeof(release_container_command), "CMD,%s,MEC,CONTAINER,ON", TEAM_ID);
	snprintf(reset_release_container_command, sizeof(reset_release_container_command), "CMD,%s,MEC,CONTAINER,OFF", TEAM_ID);
	snprintf(glider_on_command, sizeof(glider_on_command), "CMD,%s,MEC,GLIDER,ON", TEAM_ID);
	snprintf(glider_off_command, sizeof(glider_off_command), "CMD,%s,MEC,GLIDER,OFF", TEAM_ID);
	snprintf(eject_command, sizeof(eject_command), "CMD,%s,MEC,EJECT", TEAM_ID);
	snprintf(reset_state_command, sizeof(reset_state_command), "CMD,%s,RST", TEAM_ID);
	snprintf(set_coords_command, sizeof(set_coords_command), "CMD,%s,SC,", TEAM_ID);
	snprintf(set_north_command, sizeof(set_north_command), "CMD,%s,SETN", TEAM_ID);
}

char pressure_str[7];
extern uint8_t drop_detection_active;
void handle_command(const char *cmd) {
	// SIM command
	if (strncmp(cmd, sim_command, strlen(sim_command)) == 0) {

		// disable
		if (cmd[13] == 'D'){
			set_cmd_echo("SIMDISABLE", &telemetry);
			if (telemetry.mode == 'S'){
				reset_state(&telemetry);
			}
			telemetry.mode = 'F';
			telemetry.sim_enabled = 0;
		}

		// enable
		if (cmd[13] == 'E'){
			set_cmd_echo("SIMENABLE", &telemetry);
			telemetry.sim_enabled = 1;
		}

		// activate
		if (cmd[13] == 'A' && telemetry.sim_enabled == 1){
			telemetry.mode = 'S';
			set_cmd_echo("SIMACTIVATE", &telemetry);
			reset_state(&telemetry);
		}

	}

	// SIMP command
	else if (strncmp(cmd, simp_command, strlen(simp_command)) == 0) {
		if (telemetry.mode == 'S') {

			strncpy(pressure_str, &cmd[14], 6);
			pressure_str[6] = '\0';

			telemetry.pressure = atof(pressure_str)/1000;
			telemetry.altitude = calculate_altitude(&telemetry);
			telemetry.max_altitude = fmaxf(telemetry.max_altitude, telemetry.altitude);

			char temp[12] = "SIMP";
			strcat(temp, pressure_str);
			set_cmd_echo(temp, &telemetry);
			memset(pressure_str, '\0', sizeof(pressure_str));
		}
	}

	// set time command
	else if (strncmp(cmd, set_time_command, strlen(set_time_command)) == 0) {
//		drop_detection_active = 1; // TODO: REMOVE THIS
//		set_cmd_echo("DROP!!!", &telemetry); // TODO: REMOVE THIS

		if (cmd[12]=='G') {
			telemetry.mission_time_hr = telemetry.gps_time_hr;
			telemetry.mission_time_min = telemetry.gps_time_min;
			telemetry.mission_time_sec = telemetry.gps_time_sec;
			set_cmd_echo("STGPS", &telemetry);
		}
		else {
			char temp[3];
			memset(temp, 0, sizeof(temp));
			temp[0] = cmd[12];
			temp[1] = cmd[13];
			telemetry.mission_time_hr = atoi(temp);
			memset(temp, 0, sizeof(temp));
			temp[0] = cmd[15];
			temp[1] = cmd[16];
			telemetry.mission_time_min = atoi(temp);
			memset(temp, 0, sizeof(temp));
			temp[0] = cmd[18];
			temp[1] = cmd[19];
			telemetry.mission_time_sec = atoi(temp);
			memset(telemetry.cmd_echo, '\0', sizeof(telemetry.cmd_echo));
			snprintf(telemetry.cmd_echo, sizeof(telemetry.cmd_echo), "ST%02d:%02d:%02d", telemetry.mission_time_hr, telemetry.mission_time_min, telemetry.mission_time_sec);
		}

//		store_flash_data(&telemetry);
	}

	// Calibrate Altitude
	else if (strncmp(cmd, cal_alt_command, strlen(cal_alt_command)) == 0) {
		telemetry.altitude_offset -= telemetry.altitude;
		set_cmd_echo("CAL", &telemetry);
		telemetry.baro_vz = 0;
		telemetry.velocity_world_z = 0;
		telemetry.sim_enabled = 0;

		store_flash_data(&telemetry);
	}

	// Telemetry On
	else if (strncmp(cmd, tel_on_command, strlen(tel_on_command)) == 0) {
		telemetry.telemetry_status = 1;
		set_cmd_echo("CXON", &telemetry);
		telemetry.sim_enabled = 0;
		flush_gps(&hi2c1, &telemetry);
	}

	// Telemetry Off
	else if (strncmp(cmd, tel_off_command, strlen(tel_off_command)) == 0) {
		telemetry.telemetry_status = 0;
		set_cmd_echo("CXOFF", &telemetry);
		telemetry.sim_enabled = 0;
	}

	// Release Payload
	else if (strncmp(cmd, release_payload_command, strlen(release_payload_command)) == 0) {
		// Update variable
		set_cmd_echo("MECPAYLOADON", &telemetry);
		Release_Payload();
		telemetry.payload_released = 1;
		telemetry.sim_enabled = 0;

//		store_flash_data(&telemetry);
	}

	// Reset Payload Release
	else if (strncmp(cmd, reset_release_payload_command, strlen(reset_release_payload_command)) == 0) {
		// Update variable
		set_cmd_echo("MECPAYLOADOFF", &telemetry);
		Reset_Payload();
		telemetry.payload_released = 0;
		telemetry.sim_enabled = 0;
	}

	// Release Container
	else if (strncmp(cmd, release_container_command, strlen(release_container_command)) == 0) {
		// Update variable
		set_cmd_echo("MECCONTON", &telemetry);
		Release_Container();
		telemetry.container_released = 1;
		telemetry.paraglider_ejected = 0;
		telemetry.sim_enabled = 0;

//		store_flash_data(&telemetry);
	}

	// Reset Container Release
	else if (strncmp(cmd, reset_release_container_command, strlen(reset_release_container_command)) == 0) {
		// Update variable
		set_cmd_echo("MECCONTOFF", &telemetry);
		Reset_Container();
		telemetry.container_released = 0;
		telemetry.paraglider_ejected = 0;
		telemetry.sim_enabled = 0;

//		store_flash_data(&telemetry);
	}

	// Eject paraglider
	else if (strncmp(cmd, eject_command, strlen(eject_command)) == 0) {
		// Update variable
		set_cmd_echo("MECEJECT", &telemetry);
		Eject_Paraglider();
		telemetry.container_released = 1;
		telemetry.paraglider_ejected = 1;
		telemetry.sim_enabled = 0;

//		store_flash_data(&telemetry);
	}

	// Glider On
	else if (strncmp(cmd, glider_on_command, strlen(glider_on_command)) == 0) {
		// Update variable
		set_cmd_echo("MECGLIDERON", &telemetry);
		telemetry.paraglider_active = 1;
		telemetry.sim_enabled = 0;

//		store_flash_data(&telemetry);
	}

	// Glider Off
	else if (strncmp(cmd, glider_off_command, strlen(glider_off_command)) == 0) {
		// Update variable
		set_cmd_echo("MECGLIDEROFF", &telemetry);
		telemetry.paraglider_active = 0;
		telemetry.sim_enabled = 0;

//		store_flash_data(&telemetry);
	}

	// Reset State
	else if (strncmp(cmd, reset_state_command, strlen(reset_state_command)) == 0) {
		// Update variable
		set_cmd_echo("RST", &telemetry);
		reset_state(&telemetry);
		telemetry.sim_enabled = 0;
	}

	// Set Coordinates
	else if (strncmp(cmd, set_coords_command, strlen(set_coords_command)) == 0) {
		// Update variable
		set_cmd_echo("SC", &telemetry);
		sscanf(&cmd[12], "%f,%f", &telemetry.target_latitude, &telemetry.target_longitude);
		telemetry.sim_enabled = 0;

		store_flash_data(&telemetry);
	}

	// Set North
	else if (strncmp(cmd, set_north_command, strlen(set_north_command)) == 0) {
		set_cmd_echo("SETN", &telemetry);
		telemetry.sim_enabled = 0;
		set_north(&telemetry);

		store_flash_data(&telemetry);
	}
}
