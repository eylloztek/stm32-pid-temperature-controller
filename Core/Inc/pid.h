/*
 * pid.h
 *
 *  Created on: Jun 19, 2026
 *      Author: Eylül Öztek
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"

/**
 * @brief PID controller handle structure.
 *
 * This structure stores all gain parameters, setpoint information,
 * internal controller states and output limits required for PID control.
 */
typedef struct {
	float Kp; /**< Proportional gain. */
	float Ki; /**< Integral gain. */
	float Kd; /**< Derivative gain. */
	float setPoint; /**< Desired reference value. */
	float integral; /**< Accumulated integral term. */
	float prevError; /**< Previous control error used for derivative calculation. */
	float output; /**< Current PID controller output value. */
	float minOutput; /**< Minimum output limit. */
	float maxOutput; /**< Maximum output limit. */
	float sampleTime; /**< Sampling period in seconds. */
} PID_Controller_t;

/**
 * @brief Initializes a PID controller instance.
 *
 * This function sets the PID gains, sampling time, output limits and resets
 * the internal controller states.
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
		float sampleTime, float minOutput, float maxOutput);

/**
 * @brief Sets the reference value of the PID controller.
 *
 * @param pid Pointer to the PID controller structure.
 * @param setPoint Desired reference value.
 */
void PID_SetPoint(PID_Controller_t *pid, float setPoint);

/**
 * @brief Gets the current reference value of the PID controller.
 *
 * @param pid Pointer to the PID controller structure.
 * @return Current setpoint value.
 */
float PID_GetSetPoint(PID_Controller_t *pid);

/**
 * @brief Computes the PID controller output.
 *
 * This function calculates the proportional, integral and derivative terms
 * based on the current actual value and returns the limited PID output.
 *
 * @param pid Pointer to the PID controller structure.
 * @param actualValue Current measured process value.
 * @return Calculated PID output value.
 */
float PID_Compute(PID_Controller_t *pid, float actualValue);

/**
 * @brief Resets the internal PID controller states.
 *
 * This function clears the accumulated integral term and previous error value.
 *
 * @param pid Pointer to the PID controller structure.
 */
void PID_Reset(PID_Controller_t *pid);

/**
 * @brief Gets the proportional gain value.
 *
 * @param pid Pointer to the PID controller structure.
 * @return Current proportional gain.
 */
float PID_GetKp(PID_Controller_t *pid);

/**
 * @brief Gets the integral gain value.
 *
 * @param pid Pointer to the PID controller structure.
 * @return Current integral gain.
 */
float PID_GetKi(PID_Controller_t *pid);

/**
 * @brief Gets the derivative gain value.
 *
 * @param pid Pointer to the PID controller structure.
 * @return Current derivative gain.
 */
float PID_GetKd(PID_Controller_t *pid);

/**
 * @brief Sets the proportional gain value.
 *
 * @param pid Pointer to the PID controller structure.
 * @param Kp New proportional gain.
 */
void PID_SetKp(PID_Controller_t *pid, float Kp);

/**
 * @brief Sets the integral gain value.
 *
 * @param pid Pointer to the PID controller structure.
 * @param Ki New integral gain.
 */
void PID_SetKi(PID_Controller_t *pid, float Ki);

/**
 * @brief Sets the derivative gain value.
 *
 * @param pid Pointer to the PID controller structure.
 * @param Kd New derivative gain.
 */
void PID_SetKd(PID_Controller_t *pid, float Kd);

#endif /* INC_PID_H_ */
