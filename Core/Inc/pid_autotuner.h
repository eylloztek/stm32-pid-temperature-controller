/*
 * pid_autotuner.h
 *
 *  Created on: Jun 29, 2026
 *      Author: Eylül Öztek
 */

#ifndef INC_PID_AUTOTUNER_H_
#define INC_PID_AUTOTUNER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @file pid_autotuner.h
 * @brief Relay-based PID auto-tuner for STM32 temperature control projects.
 *
 * @details
 * During auto-tuning, the normal PID controller should be disabled and the
 * actuator output should be controlled by this module. The module toggles the
 * output between minOutput and maxOutput around a target input value, measures
 * the resulting process oscillation, estimates Ku and Tu, and calculates Kp,
 * Ki, and Kd values.
 *
 * @note The calculated gains are compatible with a PID implementation where:
 * integral += error * sampleTime,
 * integralTerm = Ki * integral,
 * derivativeTerm = Kd * (error - previousError) / sampleTime.
 */

#ifndef PID_AUTOTUNER_PI
#define PID_AUTOTUNER_PI 3.14159265358979323846f
#endif

typedef enum {
	PID_AUTOTUNER_OK = 0,
	PID_AUTOTUNER_ERROR,
	PID_AUTOTUNER_INVALID_PARAM,
	PID_AUTOTUNER_RUNNING,
	PID_AUTOTUNER_FINISHED
} PIDAutotuner_Status_t;

typedef enum {
	PID_AUTOTUNER_ZN_BASIC_PID = 0,
	PID_AUTOTUNER_ZN_LESS_OVERSHOOT,
	PID_AUTOTUNER_ZN_NO_OVERSHOOT
} PIDAutotuner_ZNMode_t;

typedef struct {
	float targetInputValue;
	float minOutput;
	float maxOutput;
	float outputValue;

	uint8_t outputState;
	uint8_t isRunning;
	uint8_t isFinished;

	uint16_t cycleIndex;
	uint16_t targetCycles;

	uint32_t t1_ms;
	uint32_t t2_ms;
	uint32_t tHigh_ms;
	uint32_t tLow_ms;

	float maxInput;
	float minInput;

	float ku;
	float tu_s;

	float kp;
	float ki;
	float kd;

	float kpAverage;
	float kiAverage;
	float kdAverage;
	uint16_t averageCount;

	float loopInterval_s;

	PIDAutotuner_ZNMode_t znMode;
} PIDAutotuner_t;

PIDAutotuner_Status_t PIDAutotuner_Init(PIDAutotuner_t *tuner,
		PIDAutotuner_ZNMode_t znMode, float targetInputValue, float minOutput,
		float maxOutput, uint16_t targetCycles, float loopInterval_s);

PIDAutotuner_Status_t PIDAutotuner_Start(PIDAutotuner_t *tuner,
		uint32_t currentTime_ms);

PIDAutotuner_Status_t PIDAutotuner_Stop(PIDAutotuner_t *tuner);

float PIDAutotuner_Run(PIDAutotuner_t *tuner, float input,
		uint32_t currentTime_ms);

uint8_t PIDAutotuner_IsFinished(const PIDAutotuner_t *tuner);
uint8_t PIDAutotuner_IsRunning(const PIDAutotuner_t *tuner);

float PIDAutotuner_GetKp(const PIDAutotuner_t *tuner);
float PIDAutotuner_GetKi(const PIDAutotuner_t *tuner);
float PIDAutotuner_GetKd(const PIDAutotuner_t *tuner);
float PIDAutotuner_GetKu(const PIDAutotuner_t *tuner);
float PIDAutotuner_GetTu(const PIDAutotuner_t *tuner);

uint16_t PIDAutotuner_GetCycleIndex(const PIDAutotuner_t *tuner);
uint16_t PIDAutotuner_GetTargetCycles(const PIDAutotuner_t *tuner);

#ifdef __cplusplus
}
#endif

#endif /* INC_PID_AUTOTUNER_H_ */
