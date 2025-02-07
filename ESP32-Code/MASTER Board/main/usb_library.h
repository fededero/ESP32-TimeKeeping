/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file    usb_library.h
* @brief   This file contains all the function prototypes for
*          usb_library.c file used by Dynamis PRC
******************************************************************************
* @attention
*
* Copyright (c) Dynamis PRC
* All rights reserved.
*
******************************************************************************
*******************************************************************************
 To use this file, you MUST define in the main.h file:
 1) the CAN number you are using
 2) the usb peripherals you have on the board
 in the form:
 #define _CAN_NUMBER_        //ex. _CAN_NUMBER_ = _CAN104D (the letter D is for specify that is a debug CAN)

 Example:
 #define _CAN104D_

******************************************************************************/
#ifndef _USB_LIBRARY_H_
#define _USB_LIBRARY_H_
#ifdef  __cplusplus

#endif
/* Includes ------------------------------------------------------------*/
#ifndef MATLAB_MEX_FILE
#include "main.h"
#else
#include <stdint.h>
#endif

/* SHARED DLC DEFINES */
#define CAN_DLC_BYTES_0   ((uint32_t)0)  /*!< 0 bytes data field  */
#define CAN_DLC_BYTES_1   ((uint32_t)1)  /*!< 1 bytes data field  */
#define CAN_DLC_BYTES_2   ((uint32_t)2)  /*!< 2 bytes data field  */
#define CAN_DLC_BYTES_3   ((uint32_t)3)  /*!< 3 bytes data field  */
#define CAN_DLC_BYTES_4   ((uint32_t)4)  /*!< 4 bytes data field  */
#define CAN_DLC_BYTES_5   ((uint32_t)5)  /*!< 5 bytes data field  */
#define CAN_DLC_BYTES_6   ((uint32_t)6)  /*!< 6 bytes data field  */
#define CAN_DLC_BYTES_7   ((uint32_t)7)  /*!< 7 bytes data field  */
#define CAN_DLC_BYTES_8   ((uint32_t)8)  /*!< 8 bytes data field  */
#define CAN_DLC_BYTES_12  ((uint32_t)8)  /*!< 8 bytes data field */
#define CAN_DLC_BYTES_16  ((uint32_t)8)  /*!< 8 bytes data field */
#define CAN_DLC_BYTES_20  ((uint32_t)8)  /*!< 8 bytes data field */
#define CAN_DLC_BYTES_24  ((uint32_t)8)  /*!< 8 bytes data field */
#define CAN_DLC_BYTES_32  ((uint32_t)8)  /*!< 8 bytes data field */
#define CAN_DLC_BYTES_48  ((uint32_t)8)  /*!< 8 bytes data field */
#define CAN_DLC_BYTES_64  ((uint32_t)8)  /*!< 8 bytes data field */
#ifdef _CAN100D_
#include "CAN100D.h"
#endif

/* SHARED USB CAN PACKET STRUCT */
typedef struct
{
uint8_t canID;
uint8_t DLC;
uint8_t msgID[2];
uint8_t packetData[8];
} USB_TxPacket_t;

/* CAN100D CAN STRUCT */

#ifdef _CAN100D_

typedef struct{
	uint32_t LapNumber;
	uint32_t LapTime;
}	CAN100D_Lap_struct;

typedef struct{
	CAN100D_Lap_struct	Lap;
}	CAN100D_struct_t;
#endif

/* FUNCTIONS' PROTOTYPES */

/* Prototype ------------------------------------------------------------------*/

#ifdef _CAN100D_
extern CAN100D_struct_t CAN100D_out;
void CAN100D_TX(uint16_t ID);
void CAN100D_SendUSBPacket(USB_TxPacket_t packet);
#endif
void sendDataToUSB(uint8_t * buffer, uint8_t len);


#endif // _CAN_LIBRARY_H_
