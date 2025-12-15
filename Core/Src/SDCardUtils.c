/*
 * SDCardUtils.c
 *
 *  Created on: June 7, 2024
 *      Author: Danijel Tomic & Matic Vernik
 */

#include "SDCardUtils.h"
#include "string.h"
#include <stdio.h>

extern uint64_t secondsCounter;

uint32_t GetMiliseconds(TIM_HandleTypeDef *htim) {
	return secondsCounter * 10000 + htim->Instance->CNT; // 10000 * 100000  -> 1 000 000 000
}

int countFilesOnSDCard(const char *path) {
	FRESULT res;
	DIR dir;
	FILINFO fno;
	int file_count = 0;

	res = f_opendir(&dir, path); // Open the directory
	if (res == FR_OK) {
		for (;;) {
			res = f_readdir(&dir, &fno); // Read a directory item
			if (res != FR_OK || fno.fname[0] == 0)
				break; // Break on error or end of directory
			if (fno.fattrib & AM_DIR) {
				// It is a directory
				if (!(strcmp(fno.fname, ".") == 0
						|| strcmp(fno.fname, "..") == 0)) {
					// Skip dot entries
					char next_path[256];
					snprintf(next_path, sizeof(next_path), "%s/%s", path,
							fno.fname);
					int subdir_count = countFilesOnSDCard(next_path); // Count files in the subdirectory
					if (subdir_count == -1) {
						return -1; // Propagate error
					}
					file_count += subdir_count;
				}
			} else {
				// It is a file
				file_count++;
			}
		}
		f_closedir(&dir);
	} else {
		return -1; // Return error if unable to open directory
	}

	return file_count;
}

FIL file;
uint8_t fileClosed = 1;
uint8_t firstStart = 1;

FRESULT FlushLogFile(const char *fileNameBuf) {

// Flush buffered data to SD card
	FRESULT res = f_sync(&file);
	res = f_close(&file);
	res = f_open(&file, fileNameBuf, FA_OPEN_APPEND | FA_WRITE);

	return res;
}

FRESULT initLogging(char *fileNameBuf, FATFS *fs) {
	FRESULT res;
	int file_count;
	char file_name[32];
	if (firstStart == 0) {
		f_mount(0, "", 0);
	}

// Mount the file system
	res = f_mount(fs, "", 1);
	if (res != FR_OK) {
		return res;
	}

// Count the number of files on the SD card
	file_count = countFilesOnSDCard("/");
	if (file_count == -1) {
		//f_mount(0, "", 0);
		return FR_INT_ERR;
	}

// Create a new file name based on the number of files
	if (firstStart == 0) {
		snprintf(file_name, sizeof(file_name), "logAdv%04d.csv",
				file_count + 1);
	} else {
		snprintf(file_name, sizeof(file_name), "log%04d.csv", file_count + 1);

	}

// Open the new file
	res = f_open(&file, file_name, FA_CREATE_ALWAYS | FA_WRITE);
	if (res != FR_OK) {
		//f_mount(0, "", 0);
		return res;
	}

// Write the CSV header
	const char *header = "timestamp_us,sensor_id,value\n";
	UINT bytes_written;
	res = f_write(&file, header, strlen(header), &bytes_written);
	if (res != FR_OK || bytes_written != strlen(header)) {
		f_close(&file);
		//f_mount(0, "", 0);
		return res;
	}

// Save the file name
	strcpy(fileNameBuf, file_name);

// Close the file
	f_close(&file);

	res = f_open(&file, fileNameBuf, FA_OPEN_APPEND | FA_WRITE);

	if (res != FR_OK) {
		f_close(&file);
		//f_mount(0, "", 0);
		return res;
	}

	fileClosed = 0;
	firstStart = 0;

// Unmount the file system
//	f_mount(0, "", 0);

	return FR_OK;
}
uint8_t flush_counter = 0;

FRESULT LogData(FATFS *fs, TIM_HandleTypeDef *htim, const char *fileNameBuf,
		const char *sensorIdBuff, float SensorValue) {
	FRESULT res;
//	FIL file;
	UINT bytes_written;
	char log_entry[512] = { '\0' };

// Get the current time in seconds with fractional part
	uint32_t timestamp = GetMiliseconds(htim);

// Format the log entry
	int length = snprintf(log_entry, sizeof(log_entry), "%lu00000,%s,%.2f\n",
			timestamp, sensorIdBuff, SensorValue);
	if (length < 0 || length >= sizeof(log_entry)) {
		return FR_INT_ERR; // Formatting error
	}

// Mount the file system
//	res = f_mount(fs, "", 1);
//	if (res != FR_OK) {
//		return res;
//	}

// Open the log file in append mode
	if (fileClosed) {
		res = f_open(&file, fileNameBuf, FA_OPEN_APPEND | FA_WRITE);
		if (res != FR_OK) {
			//f_mount(0, "", 0);
			return res;
		}
		fileClosed = 0;
	}

// Write the log entry to the file
	res = f_write(&file, log_entry, strlen(log_entry), &bytes_written);
	if (res != FR_OK || bytes_written != strlen(log_entry)) {
		f_close(&file);
		//f_mount(0, "", 0);
		fileClosed = 1;
		return res;
	}
	if (++flush_counter >= 50) {
		FlushLogFile(fileNameBuf);
		flush_counter = 0;
	}
// Close the file
//	f_close(&file);

// flush the file system

	return FR_OK;
}

FRESULT LogMultyData(FATFS *fs, const char *fileNameBuf, const char *logedLine) {
	FRESULT res;
//	FIL file;
	UINT bytes_written;

// Mount the file system
//	res = f_mount(fs, "", 1);
//	if (res != FR_OK) {
//		return res;
//	}

// Open the log file in append mode
	if (fileClosed) {
		res = f_open(&file, fileNameBuf, FA_OPEN_APPEND | FA_WRITE);
		if (res != FR_OK) {
			//f_mount(0, "", 0);
			return res;
		}
		fileClosed = 0;
	}

// Write the log entry to the file
	res = f_write(&file, logedLine, strlen(logedLine), &bytes_written);
	if (res != FR_OK || bytes_written != strlen(logedLine)) {
		f_close(&file);
		//f_mount(0, "", 0);
		fileClosed = 1;
		return res;
	}

// Close the file
//	f_close(&file);

// Unmount the file system
//f_mount(0, "", 0);

	return FR_OK;
}

// Log suspension sensor data
FRESULT LogSuspension(FATFS *fs, TIM_HandleTypeDef *htim, const char *fileName,
		float front_left, float front_right, float rear_left, float rear_right) {
	char log_line[512];
	uint64_t timestamp = GetMiliseconds(htim);

	int length =
			snprintf(log_line, sizeof(log_line),
					"%lld,suspension,\"{\"\"front_left\":%.1f,\"\"front_right\":%.1f,\"\"rear_left\":%.1f,\"\"rear_right\":%.1f}\"",
					timestamp, front_left, front_right, rear_left, rear_right);

	if (length < 0 || length >= sizeof(log_line)) {
		return FR_INT_ERR;
	}

	return LogMultyData(fs, fileName, log_line);
}

// Log water sensors data
FRESULT LogWaterSensors(FATFS *fs, TIM_HandleTypeDef *htim,
		const char *fileName, float temp1, float temp2, float pressure1,
		float pressure2) {
	char log_line[512] = { '\0' };
	uint32_t timestamp = GetMiliseconds(htim);

// Convert to microseconds if needed (multiply by 1000)
// Or use timestamp directly if GetMiliseconds() returns microseconds
	int length =
			snprintf(log_line, sizeof(log_line),
					"%lu00000,water_sensors,\"{\"\"temp1\"\":%.1f,\"\"temp2\"\":%.1f,\"\"pressure1\"\":%.1f,\"\"pressure2\"\":%.1f}\"\n",
					timestamp, temp1, temp2, pressure1, pressure2);

	if (length < 0 || length >= sizeof(log_line)) {
		return FR_INT_ERR;
	}

	return LogMultyData(fs, fileName, log_line);
}

// Log IMU sensor data
FRESULT LogIMU(FATFS *fs, TIM_HandleTypeDef *htim, const char *fileName,
		float accel_x, float accel_y, float accel_z, float gyro_x, float gyro_y,
		float gyro_z) {
	char log_line[512];
	uint64_t timestamp = GetMiliseconds(htim);

	int length =
			snprintf(log_line, sizeof(log_line),
					"%lld,imu,\"{\"\"accel_x\":%.2f,\"\"accel_y\":%.2f,\"\"accel_z\":%.2f,\"\"gyro_x\":%.3f,\"\"gyro_y\":%.3f,\"\"gyro_z\":%.3f}\"",
					timestamp, accel_x, accel_y, accel_z, gyro_x, gyro_y,
					gyro_z);

	if (length < 0 || length >= sizeof(log_line)) {
		return FR_INT_ERR;
	}

	return LogMultyData(fs, fileName, log_line);
}

// Log GPS data
FRESULT LogGPS(FATFS *fs, TIM_HandleTypeDef *htim, const char *fileName,
		double latitude, double longitude, float altitude, float speed,
		float heading) {
	char log_line[512];
	uint32_t timestamp = GetMiliseconds(htim);

	int length =
			snprintf(log_line, sizeof(log_line),
					"%lu00000,gps,\"{\"\"latitude\"\":%.6f,\"\"longitude\"\":%.6f,\"\"altitude\"\":%.1f,\"\"speed\"\":%.1f,\"\"heading\"\":%.1f}\"\n",
					timestamp, latitude, longitude, altitude, speed, heading);

	if (length < 0 || length >= sizeof(log_line)) {
		return FR_INT_ERR;
	}

	return LogMultyData(fs, fileName, log_line);
}

// Log gas pedal data
FRESULT LogGasPedal(FATFS *fs, TIM_HandleTypeDef *htim, const char *fileName,
		float apps1_voltage, float apps2_voltage, float pedal_percent) {
	char log_line[512] = { '\0' };
	uint32_t timestamp = GetMiliseconds(htim);

// Convert to microseconds if needed (multiply by 1000)
// Or use timestamp directly if GetMiliseconds() returns microseconds

	int length =
			snprintf(log_line, sizeof(log_line),
					"%lu00000,gas_pedal,\"{\"\"APPS1\"\":%.2f,\"\"APPS2\"\":%.2f,\"\"percent\"\":%.1f}\"\n",
					timestamp, apps1_voltage, apps2_voltage, pedal_percent);

	if (length < 0 || length >= sizeof(log_line)) {
		return FR_INT_ERR;
	}

	return LogMultyData(fs, fileName, log_line);
}

FRESULT LogHvBattery(FATFS *fs, TIM_HandleTypeDef *htim, const char *fileName,
		float max_cell_temp, float min_cell_voltage, float battery_voltage) {
	char log_line[512] = { '\0' };
	uint32_t timestamp = GetMiliseconds(htim);

// Convert to microseconds if needed (multiply by 1000)
// Or use timestamp directly if GetMiliseconds() returns microseconds

	int length =
			snprintf(log_line, sizeof(log_line),
					"%lu00000,hv_battery,\"{\"\"max_cell_temp\"\":%.2f,\"\"min_cell_voltage\"\":%.2f,\"\"battery_voltage\"\":%.1f}\"\n",
					timestamp, max_cell_temp, min_cell_voltage,
					battery_voltage);

	if (length < 0 || length >= sizeof(log_line)) {
		return FR_INT_ERR;
	}

	return LogMultyData(fs, fileName, log_line);
}

FRESULT LogLvBattery(FATFS *fs, TIM_HandleTypeDef *htim, const char *fileName,
		float max_cell_temp, float min_cell_voltage, float battery_voltage) {
	char log_line[512] = { '\0' };
	uint32_t timestamp = GetMiliseconds(htim);

// Convert to microseconds if needed (multiply by 1000)
// Or use timestamp directly if GetMiliseconds() returns microseconds

	int length =
			snprintf(log_line, sizeof(log_line),
					"%lu00000,lv_battery,\"{\"\"max_cell_temp\"\":%.2f,\"\"min_cell_voltage\"\":%.2f,\"\"battery_voltage\"\":%.1f}\"\n",
					timestamp, max_cell_temp, min_cell_voltage,
					battery_voltage);

	if (length < 0 || length >= sizeof(log_line)) {
		return FR_INT_ERR;
	}

	return LogMultyData(fs, fileName, log_line);
}

FRESULT LogHvBatteryCells(FATFS *fs, TIM_HandleTypeDef *htim,
		const char *fileName, float cells[96]) {
	char log_line[1024] = { '\0' };
	uint32_t timestamp = GetMiliseconds(htim);

// Convert to microseconds if needed (multiply by 1000)
// Or use timestamp directly if GetMiliseconds() returns microseconds

	int length = snprintf(log_line, sizeof(log_line),
			"%lu00000,hv_cells,\"{\"\"cells\"\":[", timestamp);
	for (int i = 0; i < 96; i++) {
		length += snprintf(log_line + length, sizeof(log_line) - length,
				"%.2f%s", cells[i], (i < 96 - 1) ? "," : "");
	}

// Close the JSON and string
	length += snprintf(log_line + length, sizeof(log_line) - length, "]}\"\n");

	if (length < 0 || length >= sizeof(log_line)) {
		return FR_INT_ERR;
	}

	return LogMultyData(fs, fileName, log_line);
}

FRESULT LogAIRs(FATFS *fs, TIM_HandleTypeDef *htim, const char *fileName,
		uint8_t AIRp, uint8_t AIRm) {
	char log_line[512] = { '\0' };
	uint32_t timestamp = GetMiliseconds(htim);

// Convert to microseconds if needed (multiply by 1000)
// Or use timestamp directly if GetMiliseconds() returns microseconds

	int length =
			snprintf(log_line, sizeof(log_line),
					"%lu00000,air_relays,\"{\"\"air_positive\"\":%d,\"\"air_negative\"\":%d}\"\n",
					timestamp, AIRp, AIRm);

	if (length < 0 || length >= sizeof(log_line)) {
		return FR_INT_ERR;
	}

	return LogMultyData(fs, fileName, log_line);
}

