#ifndef CAN100D_H
#define CAN100D_H

#if defined ( __CC_ARM )
#pragma anon_unions
#endif

#include <stdint.h>

#define PACK /* __attribute__((__packed__)) */

#define Lap_ID (uint16_t)0x001U


typedef struct PACK CAN100DMatrix_s {

	union PACK CAN100DLap_u {
		uint8_t _data[8];
		struct PACK {
			uint64_t LapNumber : 24;
			uint64_t LapTime : 32;
		};
		/*
		 * Lap, ID: 0x001
		 * Freq: , Rx nodes: []
		 * Signals: ['LapNumber', 'LapTime']
		 */
	} Lap;

} CAN100DMatrix;

/* ------ UNPACK ------ */
void unpack_message_can100d(CAN100DMatrix * canmatrix, uint16_t ID, uint8_t * newdata);

/* ------ PACK ------ */
void pack_message_can100d(CAN100DMatrix * canmatrix, uint16_t ID, uint8_t * outdata);

/* ------ GET ------ */
void get_message_can100d(CAN100DMatrix * canmatrix, uint16_t ID, uint8_t * DLC, uint8_t ** ptr);

/* ------ DECODES ------ */

/* --- Lap --- */
int decode_can_0x001_CAN100D_LapNumber(CAN100DMatrix* canmatrix, uint32_t* signal);
int decode_can_0x001_CAN100D_LapTime(CAN100DMatrix* canmatrix, uint32_t* signal);

/* ------ ENCODES ------ */

/* --- Lap --- */
void encode_can_0x001_CAN100D_LapNumber(CAN100DMatrix* canmatrix, uint32_t* signal);
void encode_can_0x001_CAN100D_LapTime(CAN100DMatrix* canmatrix, uint32_t* signal);


#endif
