/*! ----------------------------------------------------------------------------
 * @file    port.c
 * @brief   HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2016 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#include "port.h"

/********************************************************************************
 *
 *                              APP global variables
 *
 *******************************************************************************/

extern SPIClass* _spi;
extern uint8_t _ss;
extern uint8_t _irq;
extern uint8_t _rst;

/********************************************************************************
 *
 *                  Port private variables and function prototypes
 *
 *******************************************************************************/
// static volatile uint32_t signalResetDone;

/********************************************************************************
 *
 *                              Configuration section
 *
 *******************************************************************************/

void spi_peripheral_init(uint8_t spi_bus, uint8_t ss, uint8_t rst, uint8_t irq) {
  delay(5);
  _ss  = ss;
  _irq = irq;
  _rst = rst;

  _spi = new SPIClass(spi_bus);
  _spi->begin();
}

/********************************************************************************
 *
 *                          End of configuration section
 *
 *******************************************************************************/

/********************************************************************************
 *
 *                          DW1000 port section
 *
 *******************************************************************************/
void setup_DW1000RSTnIRQ() {
  if (_irq != 0xff) {
    pinMode(_irq, INPUT);
    // attachInterrupt(digitalPinToInterrupt(_irq), test, RISING);
  }

  if (_rst != 0xff) {
    // DW1000 data sheet v2.08 §5.6.1 page 20, the RSTn pin should not be driven high but left floating.
    pinMode(_rst, INPUT);
  }

  if (_rst == 0xff) { /* Fallback to Software Reset */
    dwt_softreset();
  } else {
    // DW1000Ng data sheet v2.08 §5.6.1 page 20, the RSTn pin should not be driven high but left floating.
    pinMode(_rst, OUTPUT);
    digitalWrite(_rst, LOW);
    delay(2); // DW1000Ng data sheet v2.08 §5.6.1 page 20: nominal 50ns, to be safe take more time
    pinMode(_rst, INPUT);
    delay(5); // dw1000Ng data sheet v1.2 page 5: nominal 3 ms, to be safe take more time
  }
}

void reset_DW1000() {
  pinMode(_ss, OUTPUT);
  digitalWrite(_ss, HIGH);

  setup_DW1000RSTnIRQ();
}
/********************************************************************************
 *
 *                              END OF IRQ section
 *
 *******************************************************************************/
