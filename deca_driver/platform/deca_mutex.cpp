/*! ----------------------------------------------------------------------------
 * @file	deca_mutex.c
 * @brief	IRQ interface / mutex implementation
 *
 * @attention
 *
 * Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */

#include "deca_device_api.h"

// ---------------------------------------------------------------------------
//
// NB: The purpose of this file is to provide for microprocessor interrupt enable/disable, this is used for
//     controlling mutual exclusion from critical sections in the code where interrupts and background
//     processing may interact.  The code using this is kept to a minimum and the disabling time is also
//     kept to a minimum, so blanket interrupt disable may be the easiest way to provide this.  But at a
//     minimum those interrupts coming from the decawave device should be disabled/re-enabled by this activity.
//
//     In porting this to a particular microprocessor, the implementer may choose to use #defines in the
//     deca_irq.h include file to map these calls transparently to the target system.  Alternatively the
//     appropriate code may be embedded in the functions provided below.
//
//     This mutex dependent on HW port.
//	   If HW port uses EXT_IRQ line to receive ready/busy status from DW1000 then mutex should use this signal
//     If HW port not use EXT_IRQ line (i.e. SW polling) then no necessary for decamutex(on/off)
//
//	   For critical section use this mutex instead
//	   __save_intstate()
//     __restore_intstate()
// ---------------------------------------------------------------------------

portMUX_TYPE deca_mutex = portMUX_INITIALIZER_UNLOCKED;

decaIrqStatus_t decamutexon(void) {
  decaIrqStatus_t s;
  s = portTRY_ENTER_CRITICAL(&deca_mutex, portMUX_NO_TIMEOUT);

  return s; // return state before disable, value is used to re-enable in decamutexoff call
}

void decamutexoff(decaIrqStatus_t s) // put a function here that re-enables the interrupt at the end of the critical section
{
  if (s == pdPASS) {                 // need to check the port state as we can't use level sensitive interrupt on the STM ARM
    portEXIT_CRITICAL(&deca_mutex);
  }
}
