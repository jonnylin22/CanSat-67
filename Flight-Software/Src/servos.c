#include "servos.h"

extern TIM_HandleTypeDef htim2;

uint8_t cur_container_servo_angle = CONTAINER_ANGLE_CLOSED;
uint8_t cur_payload_servo_angle = PAYLOAD_ANGLE_CLOSED;

void Set_Servo_Angle(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t angle) {
    // Limit the angle between 0° and 180°
    if (angle > 180) {
        angle = 180;
    }

    // Map the angle to the pulse width
//    uint32_t pulse_width = SERVO_MIN_PULSE_WIDTH +
//                           ((SERVO_MAX_PULSE_WIDTH - SERVO_MIN_PULSE_WIDTH) * angle) / 180;

    // Calculate the duty cycle for the given pulse width
    uint32_t tim_period = __HAL_TIM_GET_AUTORELOAD(htim) + 1;   // Get the timer period
    uint32_t minPulse = (tim_period * SERVO_MIN_PULSE_WIDTH) / (1000000 / SERVO_FREQUENCY);
    uint32_t maxPulse = (tim_period * SERVO_MAX_PULSE_WIDTH) / (1000000 / SERVO_FREQUENCY);
    uint32_t pulse = minPulse + (angle * (maxPulse - minPulse)) / 180;

    // Set the pulse width to TIM3 Channel 3
    __HAL_TIM_SET_COMPARE(htim, channel, pulse);
}

void Init_Servos(Telemetry_t *telemetry) {
    HAL_TIM_PWM_Start(&PAYLOAD_TIM, PAYLOAD_CHANNEL);
    HAL_TIM_PWM_Start(&CONTAINER_TIM, CONTAINER_CHANNEL);
    HAL_TIM_PWM_Start(&LEFT_SERVO_TIM, LEFT_SERVO_CHANNEL);
    HAL_TIM_PWM_Start(&RIGHT_SERVO_TIM, RIGHT_SERVO_CHANNEL);

    // Set container release to correct angle
    if (!telemetry->container_released) {
    	Set_Servo_Angle(&CONTAINER_TIM, CONTAINER_CHANNEL, CONTAINER_ANGLE_CLOSED);
    }
    else if (telemetry->paraglider_ejected){
    	Set_Servo_Angle(&CONTAINER_TIM, CONTAINER_CHANNEL, CONTAINER_ANGLE_PARAGLIDER_EJECT);
    }
    else {
    	Set_Servo_Angle(&CONTAINER_TIM, CONTAINER_CHANNEL, CONTAINER_ANGLE_OPEN);
    }

    // Set payload release to correct angle
    if (telemetry->payload_released){
    	Set_Servo_Angle(&PAYLOAD_TIM, PAYLOAD_CHANNEL, PAYLOAD_ANGLE_OPEN);
    } else {
    	Set_Servo_Angle(&PAYLOAD_TIM, PAYLOAD_CHANNEL, PAYLOAD_ANGLE_CLOSED);
    }
}

void Release_Payload(){
//	Set_Servo_Angle(&PAYLOAD_TIM, PAYLOAD_CHANNEL, PAYLOAD_ANGLE_OPEN);
	unsigned char num_partitions = 20;
	for (int i = 1; i <= num_partitions; ++i){
		Set_Servo_Angle(&PAYLOAD_TIM, PAYLOAD_CHANNEL, cur_payload_servo_angle + i * (PAYLOAD_ANGLE_OPEN - cur_payload_servo_angle) / num_partitions);
		HAL_Delay(25);
	}

	cur_payload_servo_angle = PAYLOAD_ANGLE_OPEN;
}

void Reset_Payload(){
//	Set_Servo_Angle(&PAYLOAD_TIM, PAYLOAD_CHANNEL, SERVO_ANGLE_CLOSED);
	unsigned char num_partitions = 20;
	for (int i = 1; i <= num_partitions; ++i){
		Set_Servo_Angle(&PAYLOAD_TIM, PAYLOAD_CHANNEL, cur_payload_servo_angle + i * (PAYLOAD_ANGLE_CLOSED - cur_payload_servo_angle) / num_partitions);
		HAL_Delay(25);
	}

	cur_payload_servo_angle = PAYLOAD_ANGLE_CLOSED;
}

void Release_Container(){
//	Set_Servo_Angle(&CONTAINER_TIM, CONTAINER_CHANNEL, CONTAINER_ANGLE_OPEN);
	unsigned char num_partitions = 20;
	for (int i = 1; i <= num_partitions; ++i){
		Set_Servo_Angle(&CONTAINER_TIM, CONTAINER_CHANNEL, cur_container_servo_angle + i * (CONTAINER_ANGLE_OPEN - cur_container_servo_angle) / num_partitions);
		HAL_Delay(25);
	}

	cur_container_servo_angle = CONTAINER_ANGLE_OPEN;
}

void Reset_Container(){
//	Set_Servo_Angle(&CONTAINER_TIM, CONTAINER_CHANNEL, CONTAINER_ANGLE_CLOSED);
	unsigned char num_partitions = 20;
	for (int i = 1; i <= num_partitions; ++i){
		Set_Servo_Angle(&CONTAINER_TIM, CONTAINER_CHANNEL, cur_container_servo_angle + i * (CONTAINER_ANGLE_CLOSED - cur_container_servo_angle) / num_partitions);
		HAL_Delay(25);
	}
	cur_container_servo_angle = CONTAINER_ANGLE_CLOSED;
}

void Eject_Paraglider(){
	unsigned char num_partitions = 20;
	for (int i = 1; i <= num_partitions; ++i){
		Set_Servo_Angle(&CONTAINER_TIM, CONTAINER_CHANNEL, cur_container_servo_angle + i * (CONTAINER_ANGLE_PARAGLIDER_EJECT - cur_container_servo_angle) / num_partitions);
		HAL_Delay(25);
	}
	cur_container_servo_angle = CONTAINER_ANGLE_PARAGLIDER_EJECT;
}

void Set_Left_Servo_Angle(uint8_t angle){
	Set_Servo_Angle(&LEFT_SERVO_TIM, LEFT_SERVO_CHANNEL, angle);
}

void Set_Right_Servo_Angle(uint8_t angle){
	Set_Servo_Angle(&RIGHT_SERVO_TIM, RIGHT_SERVO_CHANNEL, angle);
}
