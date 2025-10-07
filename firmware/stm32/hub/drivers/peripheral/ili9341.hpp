/*
 * ili9341.h
 *
 *  Created on: Oct 29, 2021
 *      Author: robal
 */

#ifndef INC_ILI9341_H_
#define INC_ILI9341_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "stdint.h"


void ILI9341_Init(void);
void ILI9341_Clear(uint16_t colour);
void ILI9341_SendData(uint8_t* data, uint32_t len);
void ILI9341_SetAddr(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);

#ifdef __cplusplus
}
#endif

#endif /* INC_ILI9341_H_ */
