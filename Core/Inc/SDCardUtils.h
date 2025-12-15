/*
 * SDCardUtils.h
 *
 *  Created on: May 29, 2024
 *      Author: Danijel Tomic
 */

#ifndef INC_SDCARDUTILS_H_
#define INC_SDCARDUTILS_H_
#include "fatfs.h"

int countFilesOnSDCard(const char *path);
FRESULT initLogging(char *fileNameBuf, FATFS *fs);
uint32_t GetMiliseconds(TIM_HandleTypeDef *htim);
FRESULT LogData(FATFS *fs, TIM_HandleTypeDef *htim, const char *fileNameBuf,
		const char *sensorIdBuff, float SensorValue);
FRESULT LogMultyData(FATFS *fs, const char *fileNameBuf, const char *logedLine);
FRESULT LogGasPedal(FATFS *fs, TIM_HandleTypeDef *htim, const char *fileName,
		float apps1_voltage, float apps2_voltage, float pedal_percent);
FRESULT LogHvBattery(FATFS *fs, TIM_HandleTypeDef *htim, const char *fileName,
		float max_cell_temp, float min_cell_voltage, float battery_voltage);
FRESULT LogLvBattery(FATFS *fs, TIM_HandleTypeDef *htim, const char *fileName,
		float max_cell_temp, float min_cell_voltage, float battery_voltage);
FRESULT LogHvBatteryCells(FATFS *fs, TIM_HandleTypeDef *htim,
		const char *fileName, float cells[96]);
FRESULT LogAIRs(FATFS *fs, TIM_HandleTypeDef *htim, const char *fileName,
		uint8_t AIRp, uint8_t AIRm);
FRESULT LogWaterSensors(FATFS *fs, TIM_HandleTypeDef *htim,
		const char *fileName, float temp1, float temp2, float pressure1,
		float pressure2);
FRESULT LogGPS(FATFS *fs, TIM_HandleTypeDef *htim, const char *fileName,
		double latitude, double longitude, float altitude, float speed,
		float heading);
#endif /* INC_SDCARDUTILS_H_ */
