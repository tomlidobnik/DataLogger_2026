/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartLoggingTask(void *argument);
void StartNRFTask(void *argument);
/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void StartLoggingTask(void *argument) {
	for (;;) {
		if (loggingStarted == 0) {
			loggingRes = initLogging(logFileName, &fs);
			if (loggingRes == FR_OK) {
				loggingStarted = 1;
			}
		}

		if (logNext == 1) {
			if (loggingCounter % 10 == 0) {
				if (writeErrorCounter < 45 && loggingStarted == 1) {
					loggingRes = LogGasPedal(&fs, &htim15, logFileName,
							apps1.raw, apps2.raw, apps1.value);
				}
			} else if ((loggingCounter - 1) % 10 == 0) {
				if (writeErrorCounter < 45 && loggingStarted == 1) {
					loggingRes = LogData(&fs, &htim15, logFileName,
							"brake_pressure", brakePress.value);
				}
			} else if ((loggingCounter - 2) % 10 == 0) {
				if (writeErrorCounter < 45 && loggingStarted == 1) {
					loggingRes = LogData(&fs, &htim15, logFileName,
							"steering_wheel", stearingAng.value);
				}
			} else if ((loggingCounter - 3) % 10 == 0) {
				if (writeErrorCounter < 20 && loggingStarted == 1) {
					loggingRes = LogWaterSensors(&fs, &htim15, logFileName,
							watterTemp1.value, watterTemp2.value,
							watterPress1.value, watterPress2.value);
				}
			} else if ((loggingCounter - 4) % 10 == 0) {
				if (writeErrorCounter < 45 && loggingStarted == 1) {
					loggingRes = LogHvBattery(&fs, &htim15, logFileName,
							maxHvCellTemp, minHvCellVoltage, hvVoltage);
				}
			} else if ((loggingCounter - 5) % 10 == 0) {
				if (writeErrorCounter < 45 && loggingStarted == 1) {
					loggingRes = LogLvBattery(&fs, &htim15, logFileName,
							maxLvCellTemp, minLvCellVoltage, lvVoltage);
				}
			} else if ((loggingCounter - 6) % 10 == 0) {
				if (writeErrorCounter < 45 && loggingStarted == 1) {
					loggingRes = LogData(&fs, &htim15, logFileName, "current",
							hv_current);
				}
			} else if ((loggingCounter - 7) % 10 == 0) {
				if (writeErrorCounter < 20 && loggingStarted == 1) {
					loggingRes = LogGPS(&fs, &htim15, logFileName, latitude,
							longitude, altitude, gpsSpeed, heading);
				}
			}

			if (writeErrorCounter < 45 && logAIRs == 1 && loggingStarted == 1) {
				loggingRes = LogAIRs(&fs, &htim15, logFileName, AIRpPos,
						AIRmPos);
				logAIRs = 0;
			}
			if (writeErrorCounter < 45 && logHV_Cells == 1
					&& loggingStarted == 1) {
				loggingRes = LogHvBatteryCells(&fs, &htim15, logFileName,
						hv_cells);
				logHV_Cells = 0;
			}

			loggingCounter++;
			if (loggingCounter > 100) {
				loggingCounter = 0;
			}

			if (loggingRes != FR_OK) {
				writeErrorCounter++;
			} else {
				writeErrorCounter = 0;
			}

			if (writeErrorCounter > 50) {
				loggingStarted = 0;
			}
			logNext = 0;
		}

		if (secondsCounter > 600) {
			NVIC_SystemReset();
		}

		osDelay(10); // 100Hz
	}
}

void StartNRFTask(void *argument) {
	float torque = 1.1;
	float airmFloat = 1.1;
	float airpFloat = 1.1;
	uint8_t pnum = 0;

	for (;;) {
		if (__HAL_TIM_GET_COUNTER(&htim17) >= 25) { // 10Hz
			pnum++;
			tx_completed = 0;
			if (pnum < 4) {
				rf_payload[3] = 0x00;
				memcpy(&rf_payload[4], &altitude, sizeof(float));
				memcpy(&rf_payload[8], &gpsSpeed, sizeof(float));
				memcpy(&rf_payload[12], &hv_current, sizeof(float));
				memcpy(&rf_payload[16], &latitude, sizeof(float));
				memcpy(&rf_payload[20], &longitude, sizeof(float));
				memcpy(&rf_payload[24], &directon, sizeof(float));
				memcpy(&rf_payload[28], &torque, sizeof(float));
				NRF_Upload_Payload(rf_payload, 32);
				CE_Pulse();
			} else if (pnum == 4) {
				rf_payload[3] = 0x02;
				airmFloat = (float)AIRmPos;
				airpFloat = (float)AIRpPos;
				memcpy(&rf_payload[4], &motorTemp, sizeof(float));
				memcpy(&rf_payload[8], &inverterTemp, sizeof(float));
				memcpy(&rf_payload[12], &stearingAng.value, sizeof(float));
				memcpy(&rf_payload[16], &airmFloat, sizeof(float));
				memcpy(&rf_payload[20], &airpFloat, sizeof(float));
				memcpy(&rf_payload[24], &watterTemp1.value, sizeof(float));
				memcpy(&rf_payload[28], &watterPress1.value, sizeof(float));
				NRF_Upload_Payload(rf_payload, 32);
				CE_Pulse();
			} else {
				pnum = 0;
				torque = (float) apps1.raw;
				rf_payload[3] = 0x01;
				memcpy(&rf_payload[4], &maxHvCellTemp, sizeof(float));
				memcpy(&rf_payload[8], &minHvCellVoltage, sizeof(float));
				memcpy(&rf_payload[12], &maxLvCellTemp, sizeof(float));
				memcpy(&rf_payload[16], &minLvCellVoltage, sizeof(float));
				memcpy(&rf_payload[20], &watterTemp1.value, sizeof(float));
				memcpy(&rf_payload[24], &brakePress.value, sizeof(float));
				memcpy(&rf_payload[28], &apps1.value, sizeof(float));
				NRF_Upload_Payload(rf_payload, 32);
				CE_Pulse();
			}

			TIM17->CNT = 0;
		}

		osDelay(100); // 10Hz
	}
}

/* USER CODE END Application */

