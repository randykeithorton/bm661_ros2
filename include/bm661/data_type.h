#include <stdint.h>
#include <sys/types.h>
#ifndef __DATA_TYPE_H__
#define __DATA_TYPE_H__

typedef struct bm_response_scan
{
	uint16_t angle;
	uint16_t dist1;
	uint16_t dist2;
	uint8_t  rssi1;
	uint8_t  rssi2;
} bm_response_scan_t;

#endif