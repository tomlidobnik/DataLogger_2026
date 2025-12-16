#ifndef NRF_H
#define NRF_H

#include <stdint.h>
#include "main.h"

extern uint8_t tx_completed;

void Clear_TX_DS_flag(void);
void Clear_MAX_RT_flag(void);
void NRF_Read(uint8_t REGISTER, uint8_t *buffer);
char GET_TX_FIFO_EMPTY_FLAG(void);
void CE_Pulse(void);
void NRF_Write(uint8_t REGISTER, uint8_t Data);
void NRF_Upload_Payload(uint8_t *payload, uint8_t size);
void FLUSH_TX(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif /* NRF_H */