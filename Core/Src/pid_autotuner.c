/*
 * pid_autotuner.c
 *
 *  Created on: Jun 29, 2026
 *      Author: Eylül Öztek
 */

#include "pid_autotuner.h"

static void PIDAutotuner_GetZNConstants(PIDAutotuner_ZNMode_t mode,
		float *kpConstant, float *tiConstant, float *tdConstant) {
	switch (mode) {
	case PID_AUTOTUNER_ZN_BASIC_PID:
		*kpConstant = 0.60f;
		*tiConstant = 0.50f;
		*tdConstant = 0.125f;
		break;

	case PID_AUTOTUNER_ZN_LESS_OVERSHOOT:
		*kpConstant = 0.33f;
		*tiConstant = 0.50f;
		*tdConstant = 0.33f;
		break;

	case PID_AUTOTUNER_ZN_NO_OVERSHOOT:
	default:
		*kpConstant = 0.20f;
		*tiConstant = 0.50f;
		*tdConstant = 0.33f;
		break;
	}
}

static void PIDAutotuner_CalculateGains(PIDAutotuner_t *tuner) {
	float kpConstant = 0.0f;
	float tiConstant = 0.0f;
	float tdConstant = 0.0f;

	if (tuner == 0 || tuner->tu_s <= 0.0f || tuner->loopInterval_s <= 0.0f) {
		return;
	}

	PIDAutotuner_GetZNConstants(tuner->znMode, &kpConstant, &tiConstant,
			&tdConstant);

	/*
	 * The target STM32 PID implementation already applies sampleTime inside
	 * PID_Compute(). Therefore, unlike the original Arduino implementation,
	 * Ki and Kd are not additionally scaled by loopInterval here.
	 */
	tuner->kp = kpConstant * tuner->ku;
	tuner->ki = tuner->kp / (tiConstant * tuner->tu_s);
	tuner->kd = tdConstant * tuner->kp * tuner->tu_s;
}

PIDAutotuner_Status_t PIDAutotuner_Init(PIDAutotuner_t *tuner,
		PIDAutotuner_ZNMode_t znMode, float targetInputValue, float minOutput,
		float maxOutput, uint16_t targetCycles, float loopInterval_s) {
	if (tuner == 0 || maxOutput <= minOutput || targetCycles < 3U
			|| loopInterval_s <= 0.0f) {
		return PID_AUTOTUNER_INVALID_PARAM;
	}

	tuner->targetInputValue = targetInputValue;
	tuner->minOutput = minOutput;
	tuner->maxOutput = maxOutput;
	tuner->outputValue = minOutput;

	tuner->outputState = 0U;
	tuner->isRunning = 0U;
	tuner->isFinished = 0U;

	tuner->cycleIndex = 0U;
	tuner->targetCycles = targetCycles;

	tuner->t1_ms = 0U;
	tuner->t2_ms = 0U;
	tuner->tHigh_ms = 0U;
	tuner->tLow_ms = 0U;

	tuner->maxInput = targetInputValue;
	tuner->minInput = targetInputValue;

	tuner->ku = 0.0f;
	tuner->tu_s = 0.0f;

	tuner->kp = 0.0f;
	tuner->ki = 0.0f;
	tuner->kd = 0.0f;

	tuner->kpAverage = 0.0f;
	tuner->kiAverage = 0.0f;
	tuner->kdAverage = 0.0f;
	tuner->averageCount = 0U;

	tuner->loopInterval_s = loopInterval_s;
	tuner->znMode = znMode;

	return PID_AUTOTUNER_OK;
}

PIDAutotuner_Status_t PIDAutotuner_Start(PIDAutotuner_t *tuner,
		uint32_t currentTime_ms) {
	if (tuner == 0) {
		return PID_AUTOTUNER_INVALID_PARAM;
	}

	tuner->outputState = 1U;
	tuner->isRunning = 1U;
	tuner->isFinished = 0U;
	tuner->outputValue = tuner->maxOutput;

	tuner->cycleIndex = 0U;

	tuner->t1_ms = currentTime_ms;
	tuner->t2_ms = currentTime_ms;
	tuner->tHigh_ms = 0U;
	tuner->tLow_ms = 0U;

	tuner->maxInput = tuner->targetInputValue;
	tuner->minInput = tuner->targetInputValue;

	tuner->ku = 0.0f;
	tuner->tu_s = 0.0f;

	tuner->kp = 0.0f;
	tuner->ki = 0.0f;
	tuner->kd = 0.0f;

	tuner->kpAverage = 0.0f;
	tuner->kiAverage = 0.0f;
	tuner->kdAverage = 0.0f;
	tuner->averageCount = 0U;

	return PID_AUTOTUNER_OK;
}

PIDAutotuner_Status_t PIDAutotuner_Stop(PIDAutotuner_t *tuner) {
	if (tuner == 0) {
		return PID_AUTOTUNER_INVALID_PARAM;
	}

	tuner->outputState = 0U;
	tuner->isRunning = 0U;
	tuner->isFinished = 0U;
	tuner->outputValue = tuner->minOutput;

	return PID_AUTOTUNER_OK;
}

float PIDAutotuner_Run(PIDAutotuner_t *tuner, float input,
		uint32_t currentTime_ms) {
	if (tuner == 0) {
		return 0.0f;
	}

	if (!tuner->isRunning || tuner->isFinished) {
		return tuner->outputValue;
	}

	if (input > tuner->maxInput) {
		tuner->maxInput = input;
	}

	if (input < tuner->minInput) {
		tuner->minInput = input;
	}

	if (tuner->outputState && input > tuner->targetInputValue) {
		tuner->outputState = 0U;
		tuner->outputValue = tuner->minOutput;

		tuner->t1_ms = currentTime_ms;
		tuner->tHigh_ms = tuner->t1_ms - tuner->t2_ms;

		tuner->maxInput = tuner->targetInputValue;
	}

	if (!tuner->outputState && input < tuner->targetInputValue) {
		float outputAmplitude;
		float inputAmplitude;
		uint32_t tu_ms;

		tuner->outputState = 1U;
		tuner->outputValue = tuner->maxOutput;

		tuner->t2_ms = currentTime_ms;
		tuner->tLow_ms = tuner->t2_ms - tuner->t1_ms;

		outputAmplitude = (tuner->maxOutput - tuner->minOutput) / 2.0f;
		inputAmplitude = (tuner->maxInput - tuner->minInput) / 2.0f;

		if (inputAmplitude > 0.0001f) {
			tuner->ku = (4.0f * outputAmplitude)
					/ (PID_AUTOTUNER_PI * inputAmplitude);

			tu_ms = tuner->tLow_ms + tuner->tHigh_ms;
			tuner->tu_s = (float) tu_ms / 1000.0f;

			PIDAutotuner_CalculateGains(tuner);

			if (tuner->cycleIndex > 1U) {
				tuner->kpAverage += tuner->kp;
				tuner->kiAverage += tuner->ki;
				tuner->kdAverage += tuner->kd;
				tuner->averageCount++;
			}
		}

		tuner->minInput = tuner->targetInputValue;
		tuner->cycleIndex++;
	}

	if (tuner->cycleIndex >= tuner->targetCycles) {
		tuner->outputState = 0U;
		tuner->outputValue = tuner->minOutput;
		tuner->isRunning = 0U;
		tuner->isFinished = 1U;

		if (tuner->averageCount > 0U) {
			tuner->kp = tuner->kpAverage / (float) tuner->averageCount;
			tuner->ki = tuner->kiAverage / (float) tuner->averageCount;
			tuner->kd = tuner->kdAverage / (float) tuner->averageCount;
		}
	}

	return tuner->outputValue;
}

uint8_t PIDAutotuner_IsFinished(const PIDAutotuner_t *tuner) {
	if (tuner == 0) {
		return 0U;
	}

	return tuner->isFinished;
}

uint8_t PIDAutotuner_IsRunning(const PIDAutotuner_t *tuner) {
	if (tuner == 0) {
		return 0U;
	}

	return tuner->isRunning;
}

float PIDAutotuner_GetKp(const PIDAutotuner_t *tuner) {
	if (tuner == 0) {
		return 0.0f;
	}

	return tuner->kp;
}

float PIDAutotuner_GetKi(const PIDAutotuner_t *tuner) {
	if (tuner == 0) {
		return 0.0f;
	}

	return tuner->ki;
}

float PIDAutotuner_GetKd(const PIDAutotuner_t *tuner) {
	if (tuner == 0) {
		return 0.0f;
	}

	return tuner->kd;
}

float PIDAutotuner_GetKu(const PIDAutotuner_t *tuner) {
	if (tuner == 0) {
		return 0.0f;
	}

	return tuner->ku;
}

float PIDAutotuner_GetTu(const PIDAutotuner_t *tuner) {
	if (tuner == 0) {
		return 0.0f;
	}

	return tuner->tu_s;
}

uint16_t PIDAutotuner_GetCycleIndex(const PIDAutotuner_t *tuner) {
	if (tuner == 0) {
		return 0U;
	}

	return tuner->cycleIndex;
}

uint16_t PIDAutotuner_GetTargetCycles(const PIDAutotuner_t *tuner) {
	if (tuner == 0) {
		return 0U;
	}

	return tuner->targetCycles;
}
