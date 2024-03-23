/*! ----------------------------------------------------------------------------
 * @file    deca_spi.c
 * @brief   SPI access functions
 *
 * @attention
 *
 * Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#include "deca_spi.h"

// extern SPI_HandleTypeDef hspi1; /*clocked from 72MHz*/

/********************************************************************************
 *
 *                              DW1000 SPI section
 *
 *******************************************************************************/

int openspi() {
  return 0;
} // end openspi()

int closespi(void) {
  return 0;
} // end closespi()

int writetospi(uint16_t headerLength,
               const uint8_t* headerBuffer,
               uint32_t bodyLength,
               const uint8_t* bodyBuffer) {
  //   decaIrqStatus_t stat;
  //   stat = decamutexon();

  //   while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
  //     ;

  //   HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_RESET);                   /**< Put chip select line low */

  //   HAL_SPI_Transmit(&hspi1, (uint8_t*)&headerBuffer[0], headerLength, HAL_MAX_DELAY); /* Send header in polling mode */
  //   HAL_SPI_Transmit(&hspi1, (uint8_t*)&bodyBuffer[0], bodyLength, HAL_MAX_DELAY);     /* Send data in polling mode */

  //   HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET);                     /**< Put chip select line high */

  //   decamutexoff(stat);

  return 0;
} // end writetospi()

int readfromspi(uint16_t headerLength,
                const uint8_t* headerBuffer,
                uint32_t readlength,
                uint8_t* readBuffer) {
  //   int i;
  //   decaIrqStatus_t stat;
  //   stat = decamutexon();

  //   /* Blocking: Check whether previous transfer has been finished */
  //   while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
  //     ;

  //   HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_RESET); /**< Put chip select line low */

  //   /* Send header */
  //   for (i = 0; i < headerLength; i++) {
  //     HAL_SPI_Transmit(&hspi1, &headerBuffer[i], 1, HAL_MAX_DELAY); // No timeout
  //   }

  //   /* for the data buffer use LL functions directly as the HAL SPI read function
  //    * has issue reading single bytes */
  //   while (readlength-- > 0) {
  //     /* Wait until TXE flag is set to send data */
  //     while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE) == RESET) {
  //     }

  //     hspi1.Instance->DR = 0; /* set output to 0 (MOSI), this is necessary for
  //     e.g. when waking up DW1000 from DEEPSLEEP via dwt_spicswakeup() function.
  //     */

  //     /* Wait until RXNE flag is set to read data */
  //     while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_RXNE) == RESET) {
  //     }

  //     (*readBuffer++) = hspi1.Instance->DR; // copy data read form (MISO)
  //   }

  //   HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET); /**< Put chip select line high */

  //   decamutexoff(stat);

  return 0;
} // end readfromspi()

/********************************************************************************
 *
 *                              END OF DW1000 SPI section
 *
 *******************************************************************************/
