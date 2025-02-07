/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file    usb_library.c
* @brief   This file provides code for the configuration
*          of basics USB functions used by Dynamis PRC
******************************************************************************
* @attention
*
* Copyright (c) Dynamis PRC
* All rights reserved.
*
******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "usb_library.h"
#include "string.h"
#ifdef _STM32_
#include "usbd_cdc_if.h"
#elif defined(_ESP32_)
#include "esp_log.h"
#include "driver/uart.h"
#define __weak __attribute__((weak))
#else
#define UNUSED(X) (void)X
#endif 

/* Variables ------------------------------------------------------------------*/
#ifdef _STM32_
extern USBD_HandleTypeDef hUsbDeviceFS;
#endif
#ifdef _CAN100D_
CAN100DMatrix can100d_matrix;
#endif
#ifdef _CAN100D_
CAN100D_struct_t CAN100D_out;
#endif

#ifdef _CAN100D_
/* CAN100D FUNCTIONS */

/* CAN100D_TX function */
void CAN100D_TX(uint16_t ID){
	USB_TxPacket_t CAN100D_messageOut;

	CAN100D_messageOut.canID = 100;
	memcpy(CAN100D_messageOut.msgID, &ID, 2);
	switch(ID){
		case Lap_ID:
			CAN100D_messageOut.DLC = CAN_DLC_BYTES_8;
			encode_can_0x001_CAN100D_LapNumber  (&can100d_matrix, &CAN100D_out.Lap.LapNumber);
			encode_can_0x001_CAN100D_LapTime    (&can100d_matrix, &CAN100D_out.Lap.LapTime);
			break;
		default:
			break;
	}

	pack_message_can100d(&can100d_matrix, ID, CAN100D_messageOut.packetData);
	CAN100D_SendUSBPacket(CAN100D_messageOut);
}
	#endif
void CAN100D_SendUSBPacket(USB_TxPacket_t packet)
{
	uint8_t buffer[8 + packet.DLC];
	buffer[0] = packet.canID;
	buffer[1] = packet.DLC;
	memcpy(&buffer[2], &packet.msgID, 2);
	memcpy(&buffer[8], packet.packetData, packet.DLC);
	sendDataToUSB(buffer, sizeof(buffer));
}
#if defined(_STM32_)
__weak void sendDataToUSB(uint8_t * buffer, uint8_t len){

	uint8_t sendRetry = 0;
	uint32_t time = HAL_GetTick();
	memcpy(&buffer[4], &time, 4);

	if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED){
		while(CDC_Transmit_FS(buffer, len) != USBD_OK && sendRetry < 5){
			sendRetry++;
			HAL_Delay(1);
		}
	}
}
#elif defined(_ESP32_)
__weak void sendDataToUSB(uint8_t * buffer, uint8_t len){
	uint32_t time = esp_log_timestamp();
	memcpy(&buffer[4], &time, 4);
	uart_write_bytes(UART_NUM_0, (const char *)buffer, len);
}
#else
__weak void sendDataToUSB(uint8_t * buffer, uint8_t len){

	UNUSED(buffer);
	UNUSED(len);
}
#endif