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

const SPISettings _fastSPI     = SPISettings(20000000L, SPI_MSBFIRST, SPI_MODE0);
const SPISettings _slowSPI     = SPISettings(2000000L, SPI_MSBFIRST, SPI_MODE2);
const SPISettings* _currentSPI = &_fastSPI;

SPIClass* _spi;
uint8_t _ss;
uint8_t _irq;
uint8_t _rst;

/********************************************************************************
 *
 *                              DW1000 SPI section
 *
 *******************************************************************************/

int openspi() {
  _spi->beginTransaction(*_currentSPI);
  digitalWrite(_ss, LOW);
  return 0;
} // end openspi()

int closespi(void) {
  digitalWrite(_ss, HIGH);
  _spi->endTransaction();
  return 0;
} // end closespi()

int writetospi(uint16_t headerLength, const uint8_t* headerBuffer, uint32_t bodyLength, const uint8_t* bodyBuffer) {
  decaIrqStatus_t stat;
  stat = decamutexon();
  openspi();

  _spi->transferBytes(headerBuffer, NULL, headerLength); // send header
  _spi->transferBytes(bodyBuffer, NULL, bodyLength);     // write values

  closespi();
  decamutexoff(stat);

  return 0;
} // end writetospi()

int readfromspi(uint16_t headerLength, const uint8_t* headerBuffer, uint32_t readLength, uint8_t* readBuffer) {
  decaIrqStatus_t stat;
  stat = decamutexon();
  openspi();

  _spi->transferBytes(headerBuffer, NULL, headerLength); // send header
  _spi->transferBytes(NULL, readBuffer, readLength);     // read values

  closespi();
  decamutexoff(stat);

  return 0;
} // end readfromspi()

void port_set_dw1000_fastrate(void) {
  _currentSPI = &_fastSPI;
}

void port_set_dw1000_slowrate(void) {
  _currentSPI = &_slowSPI;
}

/********************************************************************************
 *
 *                              END OF DW1000 SPI section
 *
 *******************************************************************************/
