/*! ----------------------------------------------------------------------------
 * @file    port.h
 * @brief   HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#ifndef PORT_H_
#define PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "deca_device_api.h"
#include "deca_types.h"

#include <SPI.h>
#include <esp32-hal.h>

/********************************************************************************
 *
 *                                 Types definitions
 *
 *******************************************************************************/

/********************************************************************************
 *
 *                              MACRO
 *
 *******************************************************************************/

/********************************************************************************
 *
 *                              MACRO function
 *
 *******************************************************************************/

/********************************************************************************
 *
 *                              port function prototypes
 *
 *******************************************************************************/
/**
 * @fn    portGetTickCnt()
 * @brief wrapper for to read a SysTickTimer, which is incremented with
 *        CLOCKS_PER_SEC frequency.
 *        The resolution of time32_incr is usually 1/1000 sec.
 **/
#define portGetTickCnt() xTaskGetTickCount();

void spi_peripheral_init(uint8_t spi_bus = VSPI, uint8_t ss = 0xFF, uint8_t rst = 0xFF, uint8_t irq = 0xFF);
void setup_DW1000RSTnIRQ();
void reset_DW1000();

#ifdef __cplusplus
}
#endif

#endif /* PORT_H_ */
