/*! ----------------------------------------------------------------------------
 *  @file   deca_types.h
 *  @brief  Decawave general type definitions
 *
 * @attention
 *
 * Copyright 2013 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */

#ifndef _DECA_TYPES_H_
#define _DECA_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <assert.h>
#include <esp32-hal.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

typedef enum {
  DWT_SUCCESS = 0,
  DWT_ERROR   = -1
} dwt_error_e;

#ifndef __INLINE
  #define __INLINE inline
#endif

typedef bool boolean;

#ifdef __cplusplus
}
#endif

#endif /* DECA_TYPES_H_ */
