#include <stdint.h>
#include "nrf.h"

uint8_t tx_completed = 1;

void Clear_TX_DS_flag() {
	uint8_t STATUS_REG = 0x07 | 1 << 5;
	uint8_t TX_DS_bit = 1 << 5;
	HAL_GPIO_WritePin(GPIOG, CSN_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, &STATUS_REG, 1, 100);
	HAL_SPI_Transmit(&hspi3, &TX_DS_bit, 1, 100);
	HAL_GPIO_WritePin(GPIOG, CSN_Pin, GPIO_PIN_SET);
}

void Clear_MAX_RT_flag() {
	uint8_t STATUS_REG = 0x07 | 1 << 5;
	uint8_t MAX_RT_bit = 1 << 4;
	HAL_GPIO_WritePin(GPIOG, CSN_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, &STATUS_REG, 1, 100);
	HAL_SPI_Transmit(&hspi3, &MAX_RT_bit, 1, 100);
	HAL_GPIO_WritePin(GPIOG, CSN_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
}

void NRF_Read(uint8_t REGISTER, uint8_t *buffer) {
	HAL_GPIO_WritePin(GPIOG, CSN_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, &REGISTER, 1, 100);
	HAL_SPI_Receive(&hspi3, buffer, 1, 100);
	HAL_GPIO_WritePin(GPIOG, CSN_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
}

char GET_TX_FIFO_EMPTY_FLAG() {
	uint8_t buff;
	uint8_t FIFO_STATUS = 0x17;
	HAL_GPIO_WritePin(GPIOG, CSN_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, &FIFO_STATUS, 1, 100);
	HAL_SPI_Receive(&hspi3, &buff, 1, 100);
	HAL_GPIO_WritePin(GPIOG, CSN_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	return buff;
}

void CE_Pulse() {
	HAL_GPIO_WritePin(GPIOG, CE_Pin, GPIO_PIN_SET);
	TIM16->CNT = 0;
	while (__HAL_TIM_GET_COUNTER(&htim16) < 20)
		;
	HAL_GPIO_WritePin(GPIOG, CE_Pin, GPIO_PIN_RESET);
}

void NRF_Write(uint8_t REGISTER, uint8_t Data) {
	REGISTER |= 1 << 5;
	HAL_GPIO_WritePin(GPIOG, CSN_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, &REGISTER, 1, 100);
	HAL_SPI_Transmit(&hspi3, &Data, 1, 100);
	HAL_GPIO_WritePin(GPIOG, CSN_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
}

void NRF_Upload_Payload(uint8_t *payload, uint8_t size) {
	uint8_t W_TX_PAYLOAD = 1 << 7 | 1 << 5;
	HAL_GPIO_WritePin(GPIOG, CSN_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, &W_TX_PAYLOAD, 1, 100);
	HAL_SPI_Transmit(&hspi3, payload, size, 100);
	HAL_GPIO_WritePin(GPIOG, CSN_Pin, GPIO_PIN_SET);
}

void FLUSH_TX() {
	uint8_t FLUSH_TX_COMMAND = 0xe1;
	HAL_GPIO_WritePin(GPIOG, CSN_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, &FLUSH_TX_COMMAND, 1, 100);
	HAL_GPIO_WritePin(GPIOG, CSN_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	Clear_TX_DS_flag();
	tx_completed = 1;
}