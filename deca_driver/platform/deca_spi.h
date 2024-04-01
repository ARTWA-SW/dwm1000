/*! ----------------------------------------------------------------------------
 * @file	deca_spi.h
 * @brief	SPI access functions
 *
 * @attention
 *
 * Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#ifndef _DECA_SPI_H_
#define _DECA_SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "deca_types.h"
#include "port.h"

/** ------------------------------------------------------------------------------------------------------------------
 * @fn openspi()
 *
 * @brief
 * Low level abstract function to open and initialise access to the SPI device.
 * returns 0 for success, or -1 for error
 *
 * @returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int openspi(void);

/** ------------------------------------------------------------------------------------------------------------------
 * @fn closespi()
 *
 * @brief
 * Low level abstract function to close the the SPI device.
 * returns 0 for success, or -1 for error
 *
 * @returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int closespi(void);

/** ------------------------------------------------------------------------------------------------------------------
 * @fn writetospi()
 *
 * @brief
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * return 0 for success, or -1 for error
 *
 * @note
 * In porting this to a particular microprocessor, the implementer needs to define the two low
 * level abstract functions to write to and read from the SPI the definitions should be in deca_spi.c file.
 *
 * @param[in] headerLength number of bytes header being written
 * @param[in] headerBuffer pointer to buffer containing the `headerLength` bytes of header to be written
 * @param[in] bodyLength umber of bytes data being written
 * @param[in] bodyBuffer pointer to buffer containing the `bodylength` bytes od data to be written
 *
 * @returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int writetospi(uint16_t headerLength, const uint8_t* headerBuffer, uint32_t bodyLength, const uint8_t* bodyBuffer);

/** ------------------------------------------------------------------------------------------------------------------
 * @fn readfromspi()
 *
 * @brief
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * return 0 for success, or -1 for error
 *
 * @note
 * In porting this to a particular microprocessor, the implementer needs to define the two low
 * level abstract functions to write to and read from the SPI the definitions should be in deca_spi.c file.
 *
 * @param[in] headerLength number of bytes header to write
 * @param[in] headerBuffer pointer to buffer containing the 'headerLength' bytes of header to write
 * @param[in] readLength number of bytes data being read
 * @param[out] readBuffer pointer to buffer containing to return the data
 *
 * @warning size required = headerLength + readlength
 *
 * @returns DWT_SUCCESS for success (and the position in the buffer at which data begins), or DWT_ERROR for error
 */
int readfromspi(uint16_t headerLength, const uint8_t* headerBuffer, uint32_t readLength, uint8_t* readBuffer);

/**
 * @fn      port_set_dw1000_fastrate
 * @brief   sets High SPI clock speed for the DW chip
 */
void port_set_dw1000_fastrate(void);
/**
 * @fn  port_set_dw1000_slowrate
 * @brief sets slow SPI clock speed for the DW chip
 *        left for compatibility.
 */
void port_set_dw1000_slowrate(void);

#ifdef __cplusplus
}
#endif

#endif /* _DECA_SPI_H_ */
