#pragma once

#include "stdint.h"
#include "stdbool.h"
#include "stm32l4xx_hal.h"

typedef union {
	uint64_t value;
	uint8_t bytes[8];
	uint16_t shorts[4];
	int32_t ints[2];
	uint32_t uints[2];
	float floats[2];
	double dub;
} CanPayload;

typedef struct {
	CanPayload data;
	CAN_TxHeaderTypeDef Header;
} CanMessage;

HAL_StatusTypeDef SendCanMsg1(CanMessage canMessage);

CanMessage CreateSendBrakeWater(uint16_t reqTorque,float brakePress, float waterTemp, float waterPress);

CanMessage CreateSendMotorInverter(float inverterTemp, float motorTemp);

CanMessage CreateSendCellDataRequest(void);
