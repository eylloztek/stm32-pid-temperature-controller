/*
 * pid.c
 *
 *  Created on: Jun 19, 2026
 *      Author: Eylül Öztek
 */

#include "pid.h"

/**
 * @brief Initializes a PID controller instance.
 *
 * This function assigns the proportional, integral and derivative gains,
 * sets the sampling time and output limits, and clears the internal PID states.
 *
 * @param pid Pointer to the PID controller structure.
 * @param Kp Proportional gain.
 * @param Ki Integral gain.
 * @param Kd Derivative gain.
 * @param sampleTime Sampling period in seconds.
 * @param minOutput Minimum output limit.
 * @param maxOutput Maximum output limit.
 */
void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd,
		float sampleTime, float minOutput, float maxOutput) {

	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->sampleTime = sampleTime;
	pid->minOutput = minOutput;
	pid->maxOutput = maxOutput;
	pid->integral = 0.0f;
	pid->prevError = 0.0f;
	pid->output = 0.0f;
}

/**
 * @brief Sets the desired setpoint value.
 *
 * @param pid Pointer to the PID controller structure.
 * @param setPoint Desired reference value.
 */
void PID_SetPoint(PID_Controller_t *pid, float setPoint) {

	pid->setPoint = setPoint;
}

/**
 * @brief Gets the current setpoint value.
 *
 * @param pid Pointer to the PID controller structure.
 * @return Current setpoint value.
 */
float PID_GetSetPoint(PID_Controller_t *pid) {

	return pid->setPoint;
}

/**
 * @brief Computes the PID controller output.
 *
 * This function calculates the control error using the current setpoint and
 * measured actual value. It then computes the proportional, integral and
 * derivative terms. The integral term and final output are limited between
 * the configured minimum and maximum output limits.
 *
 * @param pid Pointer to the PID controller structure.
 * @param actualValue Current measured process value.
 * @return Limited PID output value.
 */
float PID_Compute(PID_Controller_t *pid, float actualValue) {

	float error = pid->setPoint - actualValue;

	float propotional = pid->Kp * error;

	pid->integral += error * pid->sampleTime;

	if (pid->integral > pid->maxOutput) {
		pid->integral = pid->maxOutput;
	} else if (pid->integral < pid->minOutput) {
		pid->integral = pid->minOutput;
	}

	float integral = pid->Ki * pid->integral;

	float derivative = pid->Kd * (error - pid->prevError) / pid->sampleTime;
	pid->prevError = error;

	pid->output = propotional + integral + derivative;

	if (pid->output > pid->maxOutput) {
		pid->output = pid->maxOutput;
	} else if (pid->output < pid->minOutput) {
		pid->output = pid->minOutput;
	}

	return pid->output;
}

/**
 * @brief Resets the internal PID controller states.
 *
 * This function clears the accumulated integral value and previous error.
 * The configured gains, setpoint, sampling time and output limits are not changed.
 *
 * @param pid Pointer to the PID controller structure.
 */
void PID_Reset(PID_Controller_t *pid) {

	pid->integral = 0.0f;
	pid->prevError = 0.0f;
}

/**
 * @brief Gets the proportional gain value.
 *
 * @param pid Pointer to the PID controller structure.
 * @return Current proportional gain.
 */
float PID_GetKp(PID_Controller_t *pid) {
	return pid->Kp;
}

/**
 * @brief Gets the integral gain value.
 *
 * @param pid Pointer to the PID controller structure.
 * @return Current integral gain.
 */
float PID_GetKi(PID_Controller_t *pid) {
	return pid->Ki;
}

/**
 * @brief Gets the derivative gain value.
 *
 * @param pid Pointer to the PID controller structure.
 * @return Current derivative gain.
 */
float PID_GetKd(PID_Controller_t *pid) {
	return pid->Kd;
}

/**
 * @brief Sets the proportional gain value.
 *
 * @param pid Pointer to the PID controller structure.
 * @param Kp New proportional gain.
 */
void PID_SetKp(PID_Controller_t *pid, float Kp) {
	pid->Kp = Kp;
}

/**
 * @brief Sets the integral gain value.
 *
 * @param pid Pointer to the PID controller structure.
 * @param Ki New integral gain.
 */
void PID_SetKi(PID_Controller_t *pid, float Ki) {
	pid->Ki = Ki;
}

/**
 * @brief Sets the derivative gain value.
 *
 * @param pid Pointer to the PID controller structure.
 * @param Kd New derivative gain.
 */
void PID_SetKd(PID_Controller_t *pid, float Kd) {
	pid->Kd = Kd;
}
