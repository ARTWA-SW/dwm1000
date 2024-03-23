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

#include <shared/deca_types.h>

#define DECA_MAX_SPI_HEADER_LENGTH (3) // max number of bytes in header (for formating & sizing)

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
 * @param headerLength number of bytes header being written
 * @param headerBuffer pointer to buffer containing the 'headerLength' bytes of header to be written
 * @param bodylength umber of bytes data being written
 * @param bodyBuffer pointer to buffer containing the 'bodylength' bytes od data to be written
 *
 * @returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int writetospi(uint16_t headerLength, const uint8_t* headerBuffer, uint32_t bodylength, const uint8_t* bodyBuffer);

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
 * @param headerLength number of bytes header to write
 * @param headerBuffer pointer to buffer containing the 'headerLength' bytes of header to write
 * @param readlength number of bytes data being read
 * @param readBuffer pointer to buffer containing to return the data
 *
 * @warning size required = headerLength + readlength
 *
 * @returns DWT_SUCCESS for success (and the position in the buffer at which data begins), or DWT_ERROR for error
 */
int readfromspi(uint16_t headerLength, const uint8_t* headerBuffer, uint32_t readlength, uint8_t* readBuffer);

#ifdef __cplusplus
}
#endif

#endif /* _DECA_SPI_H_ */
