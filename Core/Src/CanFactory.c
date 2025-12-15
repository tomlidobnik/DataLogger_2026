#include "CanFactory.h"

extern CAN_HandleTypeDef hcan1;
HAL_StatusTypeDef SendCanMsg1(CanMessage canMessage) {
	int emptyMails = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
	if (emptyMails == 0) {
		// počaka da mailbox ni poln
		//		HAL_Delay(1);
		HAL_CAN_Stop(&hcan1);
//		HAL_Delay(10);
		HAL_CAN_Start(&hcan1);
		return HAL_ERROR;
	}
	uint32_t TxMailbox;
	HAL_StatusTypeDef res = HAL_CAN_AddTxMessage(&hcan1, &canMessage.Header,
			canMessage.data.bytes, &TxMailbox);
	return res;
}

CanMessage CreateSendBrakeWater(uint16_t reqTorque,float brakePress, float waterTemp, float waterPress) {
	CanMessage canMessage;
	canMessage.Header.StdId = 0x200;
	canMessage.Header.DLC = 8;
	canMessage.Header.RTR = CAN_RTR_DATA;
	canMessage.Header.IDE = CAN_ID_STD;

	canMessage.data.shorts[0] = (uint16_t)(reqTorque/8);
	canMessage.data.shorts[1] = (uint16_t)(brakePress);
	canMessage.data.shorts[2] = (uint16_t)(waterTemp);
	canMessage.data.shorts[3] = (uint16_t)(waterPress);
	return canMessage;
}

CanMessage CreateSendMotorInverter(float inverterTemp, float motorTemp) {
	CanMessage canMessage;
	canMessage.Header.StdId = 0x201;
	canMessage.Header.DLC = 6;
	canMessage.Header.RTR = CAN_RTR_DATA;
	canMessage.Header.IDE = CAN_ID_STD;

	canMessage.data.shorts[1] = (uint16_t)(motorTemp);
	canMessage.data.shorts[2] = (uint16_t)(inverterTemp);
	return canMessage;
}

CanMessage CreateSendCellDataRequest(void){
	CanMessage canMessage;
		canMessage.Header.StdId = 0x731;
		canMessage.Header.DLC = 0;
		canMessage.Header.RTR = CAN_RTR_DATA;
		canMessage.Header.IDE = CAN_ID_STD;

		return canMessage;
}
