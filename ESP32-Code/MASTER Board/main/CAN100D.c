#include <string.h>

#include "CAN100D.h"

/* ------ UNPACK ------ */
void unpack_message_can100d(CAN100DMatrix * canmatrix, uint16_t ID, uint8_t * newdata) {
	switch(ID) {
		case Lap_ID:
			memcpy(canmatrix->Lap._data, newdata, 8);
			break;
	}
}

/* --- Lap --- */
int decode_can_0x001_CAN100D_LapNumber(CAN100DMatrix* canmatrix, uint32_t* signal) {
	*signal = ((uint32_t)canmatrix->Lap.LapNumber);
	return 0;
}
int decode_can_0x001_CAN100D_LapTime(CAN100DMatrix* canmatrix, uint32_t* signal) {
	*signal = ((uint32_t)canmatrix->Lap.LapTime);
	return 0;
}

/* ------ PACK ------ */
void pack_message_can100d(CAN100DMatrix * canmatrix, uint16_t ID, uint8_t * outdata) {
	switch(ID) {
		case Lap_ID:
			memcpy(outdata, canmatrix->Lap._data, 8);
			break;
	}
}

/* --- Lap --- */
void encode_can_0x001_CAN100D_LapNumber(CAN100DMatrix* canmatrix, uint32_t* signal) {
	canmatrix->Lap.LapNumber = *signal;
}
void encode_can_0x001_CAN100D_LapTime(CAN100DMatrix* canmatrix, uint32_t* signal) {
	canmatrix->Lap.LapTime = *signal;
}

/* ------ GET ------ */
void get_message_can100d(CAN100DMatrix * canmatrix, uint16_t ID, uint8_t * DLC, uint8_t ** ptr) {
	switch(ID) {
		case Lap_ID:
			*DLC = 8;
			*ptr = canmatrix->Lap._data;
			break;
	}
}

