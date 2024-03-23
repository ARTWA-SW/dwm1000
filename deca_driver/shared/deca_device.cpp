/*! ------------------------------------------------------------------------------------------------------------------
 * @file    deca_device.c
 * @brief   Decawave device configuration and control functions
 *
 * @attention
 *
 * Copyright 2013 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */

#include "deca_device_api.h"
#include "deca_param_types.h"
#include "deca_regs.h"
#include "deca_types.h"

#include <assert.h>
#include <stdlib.h>

// Defines for enable_clocks function
#define FORCE_SYS_XTI  0
#define ENABLE_ALL_SEQ 1
#define FORCE_SYS_PLL  2
#define READ_ACC_ON    7
#define READ_ACC_OFF   8
#define FORCE_OTP_ON   11
#define FORCE_OTP_OFF  12
#define FORCE_TX_PLL   13
#define FORCE_LDE      14

// Defines for ACK request bitmask in DATA and MAC COMMAND frame control (first byte) - Used to detect AAT bit wrongly set.
#define FCTRL_ACK_REQ_MASK 0x20
// Frame control maximum length in bytes.
#define FCTRL_LEN_MAX      2

// #define DWT_API_ERROR_CHECK     // define so API checks config input parameters

// -------------------------------------------------------------------------------------------------------------------
//
// Internal functions for controlling and configuring the device
//
// -------------------------------------------------------------------------------------------------------------------

/** ------------------------------------------------------------------------------------------------------------------
 * @fn _dwt_otpread()
 *
 * @brief function to read the OTP memory. Ensure that MR,MRa,MRb are reset to 0.
 *
 * @param address address to read at
 *
 * @returns the 32bit of read data
 */
uint32_t _dwt_otpread(uint16_t address);

/** ------------------------------------------------------------------------------------------------------------------
 * @fn _dwt_otpsetmrregs()
 *
 * @brief Configure the MR registers for initial programming (enable charge pump).
 * Read margin is used to stress the read back from the
 * programmed bit. In normal operation this is relaxed.
 *
 * @param mode "0" : Reset all to 0x0:           MRA=0x0000, MRB=0x0000, MR=0x0000
 *             "1" : Set for inital programming: MRA=0x9220, MRB=0x000E, MR=0x1024
 *             "2" : Set for soak programming:   MRA=0x9220, MRB=0x0003, MR=0x1824
 *             "3" : High Vpp:                   MRA=0x9220, MRB=0x004E, MR=0x1824
 *             "4" : Low Read Margin:            MRA=0x0000, MRB=0x0003, MR=0x0000
 *             "5" : Array Clean:                MRA=0x0049, MRB=0x0003, MR=0x0024
 *             "4" : Very Low Read Margin:       MRA=0x0000, MRB=0x0003, MR=0x0000
 *
 * @returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
uint32_t _dwt_otpsetmrregs(int mode);

/** ------------------------------------------------------------------------------------------------------------------
 * @fn _dwt_otpprogword32()
 *
 * @brief function to program the OTP memory. Ensure that MR,MRa,MRb are reset to 0.
 *        VNM Charge pump needs to be enabled (see _dwt_otpsetmrregs)
 *
 * @note the address is only 11 bits long.
 *
 * @param address address to read at
 *
 * @returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
uint32_t _dwt_otpprogword32(uint32_t data, uint16_t address);

/** ------------------------------------------------------------------------------------------------------------------
 * @fn _dwt_aonconfigupload()
 *
 * @brief This function uploads always on (AON) configuration, as set in the AON_CFG0_OFFSET register.
 *
 * no return value
 */
void _dwt_aonconfigupload(void);

/** ------------------------------------------------------------------------------------------------------------------
 * @fn _dwt_aonarrayupload()
 *
 * @brief This function uploads always on (AON) data array and configuration. Thus if this function is used, then _dwt_aonconfigupload
 * is not necessary. The DW1000 will go so SLEEP straight after this if the DWT_SLP_EN has been set.
 *
 * no return value
 */
void _dwt_aonarrayupload(void);
// -------------------------------------------------------------------------------------------------------------------

/** ------------------------------------------------------------------------------------------------------------------
 * @fn _dwt_configlde()
 *
 * @brief configure LDE algorithm parameters
 *
 * @param prf this is the PRF index (0 or 1) 0 corresponds to 16 and 1 to 64 PRF
 *
 * no return value
 */
void _dwt_configlde(int prf);

/** ------------------------------------------------------------------------------------------------------------------
 * @fn _dwt_loaducodefromrom()
 *
 * @brief  load ucode from OTP MEMORY or ROM
 *
 * no return value
 */
void _dwt_loaducodefromrom(void);

/** ------------------------------------------------------------------------------------------------------------------
 * @fn _dwt_enableclocks()
 *
 * @brief function to enable/disable clocks to particular digital blocks/system
 *
 * @param clocks set of clocks to enable/disable
 *
 * no return value
 */
void _dwt_enableclocks(int clocks);

/** ------------------------------------------------------------------------------------------------------------------
 * @fn _dwt_disablesequencing()
 *
 * @brief This function disables the TX blocks sequencing, it disables PMSC control of RF blocks, system clock is also set to XTAL
 *
 * no return value
 */
void _dwt_disablesequencing(void);

/** ------------------------------------------------------------------------------------------------------------------
 * @fn _dwt_computetxpowersetting()
 *
 * @brief this function calculates the appropriate change to the TX_POWER register to compensate
 * the TX power output at different temperatures.
 *
 * @param ref_powerreg the TX_POWER register value recorded when reference measurements were made
 * @param power_adj the adjustment in power level to be made, in 0.5dB steps
 *
 * @returns The setting to be programmed into the TX_POWER register
 */
uint32_t _dwt_computetxpowersetting(uint32_t ref_powerreg, int32_t power_adj);

/*!
 * Static data for DW1000 DecaWave Transceiver control
 */

// -------------------------------------------------------------------------------------------------------------------
// Structure to hold device data
typedef struct
{
  uint32_t partID;      // IC Part ID - read during initialisation
  uint32_t lotID;       // IC Lot ID - read during initialisation
  uint8_t vBatP;        // IC V bat read during production and stored in OTP (Vmeas @ 3V3)
  uint8_t tempP;        // IC V temp read during production and stored in OTP (Tmeas @ 23C)
  uint8_t longFrames;   // Flag in non-standard long frame mode
  uint8_t otprev;       // OTP revision number (read during initialisation)
  uint32_t txFCTRL;     // Keep TX_FCTRL register config
  uint32_t sysCFGreg;   // Local copy of system config register
  uint8_t dblbuffon;    // Double RX buffer mode flag
  uint8_t wait4resp;    // wait4response was set with last TX start command
  uint16_t sleep_mode;  // Used for automatic reloading of LDO tune and microcode at wake-up
  uint16_t otp_mask;    // Local copy of the OTP mask used in dwt_initialise call
  dwt_cb_data_t cbData; // Callback data structure
  dwt_cb_t cbTxDone;    // Callback for TX confirmation event
  dwt_cb_t cbRxOk;      // Callback for RX good frame event
  dwt_cb_t cbRxTo;      // Callback for RX timeout events
  dwt_cb_t cbRxErr;     // Callback for RX error events
} dwt_local_data_t;

static dwt_local_data_t dw1000local[DWT_NUM_DW_DEV]; // Static local device data, can be an array to support multiple DW1000 testing applications/platforms
static dwt_local_data_t* pdw1000local = dw1000local; // Static local data structure pointer

int32_t dwt_apiversion(void) {
  return DW1000_DRIVER_VERSION;
}

int dwt_setlocaldataptr(unsigned int index) {
  // Check the index is within the array bounds
  if (DWT_NUM_DW_DEV <= index) // return error if index outside the array bounds
  {
    return DWT_ERROR;
  }

  pdw1000local = &dw1000local[index];

  return DWT_SUCCESS;
}

// OTP addresses definitions
#define LDOTUNE_ADDRESS (0x04)
#define PARTID_ADDRESS  (0x06)
#define LOTID_ADDRESS   (0x07)
#define VBAT_ADDRESS    (0x08)
#define VTEMP_ADDRESS   (0x09)
#define XTRIM_ADDRESS   (0x1E)

int dwt_initialise(int config) {
  uint16_t otp_xtaltrim_and_rev = 0;
  uint32_t ldo_tune             = 0;

  pdw1000local->dblbuffon  = 0; // - set to 0 - meaning double buffer mode is off by default
  pdw1000local->wait4resp  = 0; // - set to 0 - meaning wait for response not active
  pdw1000local->sleep_mode = 0; // - set to 0 - meaning sleep mode has not been configured

  pdw1000local->cbTxDone = NULL;
  pdw1000local->cbRxOk   = NULL;
  pdw1000local->cbRxTo   = NULL;
  pdw1000local->cbRxErr  = NULL;

#if DWT_API_ERROR_CHECK
  pdw1000local->otp_mask = config; // Save the READ_OTP config mask
#endif

  // Read and validate device ID, return -1 if not recognised
  if (DWT_DEVICE_ID != dwt_readdevid()) // MP IC ONLY (i.e. DW1000) FOR THIS CODE
  {
    return DWT_ERROR;
  }

  if (!(DWT_DW_WAKE_UP & config)) // Don't reset the device if DWT_DW_WAKE_UP bit is set, e.g. when calling this API after wake up
  {
    dwt_softreset();              // Make sure the device is completely reset before starting initialisation
  }

  if (!((DWT_DW_WAKE_UP & config) && ((DWT_READ_OTP_TMP | DWT_READ_OTP_BAT | DWT_READ_OTP_LID | DWT_READ_OTP_PID | DWT_DW_WUP_RD_OTPREV) & config))) {
    _dwt_enableclocks(FORCE_SYS_XTI); // NOTE: set system clock to XTI - this is necessary to make sure the values read by _dwt_otpread are reliable
  } // when not reading from OTP, clocks don't need to change.

  // Configure the CPLL lock detect
  dwt_write8bitoffsetreg(EXT_SYNC_ID, EC_CTRL_OFFSET, EC_CTRL_PLLLCK);

  // When DW1000 IC is initialised from power up, then the LDO value should be kicked from OTP, otherwise if this API is called after
  // DW1000 IC has been woken up (DWT_DW_WAKE_UP bit is set) this can be skipped as LDO would have already been automatically
  // kicked/loaded on wake up
  if (!(DWT_DW_WAKE_UP & config)) {
    // Load LDO tune from OTP and kick it if there is a value actually programmed.
    ldo_tune = _dwt_otpread(LDOTUNE_ADDRESS);
    if ((ldo_tune & 0xFF) != 0) {
      // Kick LDO tune
      dwt_write8bitoffsetreg(OTP_IF_ID, OTP_SF, OTP_SF_LDO_KICK); // Set load LDO kick bit
      pdw1000local->sleep_mode |= AON_WCFG_ONW_LLDO;              // LDO tune must be kicked at wake-up
    }
  } else {                                                        // if LDOTUNE reg contains value different from default it means it was kicked from OTP and thus set AON_WCFG_ONW_LLDO.
    if (dwt_read32bitoffsetreg(RF_CONF_ID, LDOTUNE) != LDOTUNE_DEFAULT)
      pdw1000local->sleep_mode |= AON_WCFG_ONW_LLDO;
  }

  if ((!(DWT_DW_WAKE_UP & config)) || ((DWT_DW_WAKE_UP & config) && (DWT_DW_WUP_RD_OTPREV & config))) {
    // Read OTP revision number
    otp_xtaltrim_and_rev = _dwt_otpread(XTRIM_ADDRESS) & 0xffff; // Read 32 bit value, XTAL trim val is in low octet-0 (5 bits)
    pdw1000local->otprev = (otp_xtaltrim_and_rev >> 8) & 0xff;   // OTP revision is the next byte
  } else
    pdw1000local->otprev = 0;                                    // If OTP valuse are not used, if this API is called after DW1000 IC has been woken up
                                                                 // (DWT_DW_WAKE_UP bit is set), set otprev to 0

  if (!(DWT_DW_WAKE_UP & config)) {
    // XTAL trim value is set in OTP for DW1000 module and EVK/TREK boards but that might not be the case in a custom design
    if ((otp_xtaltrim_and_rev & 0x1F) == 0)     // A value of 0 means that the crystal has not been trimmed
    {
      otp_xtaltrim_and_rev = FS_XTALT_MIDRANGE; // Set to mid-range if no calibration value inside
    }
    // Configure XTAL trim
    dwt_setxtaltrim((uint8_t)otp_xtaltrim_and_rev);
  }

  if (DWT_READ_OTP_PID & config) {
    // Load Part from OTP
    pdw1000local->partID = _dwt_otpread(PARTID_ADDRESS);
  } else {
    pdw1000local->partID = 0;
  }

  if (DWT_READ_OTP_LID & config) {
    // Load Lot ID from OTP
    pdw1000local->lotID = _dwt_otpread(LOTID_ADDRESS);
  } else {
    pdw1000local->lotID = 0;
  }

  if (DWT_READ_OTP_BAT & config) {
    // Load VBAT from OTP
    pdw1000local->vBatP = _dwt_otpread(VBAT_ADDRESS) & 0xff;
  } else {
    pdw1000local->vBatP = 0;
  }

  if (DWT_READ_OTP_TMP & config) {
    // Load TEMP from OTP
    pdw1000local->tempP = _dwt_otpread(VTEMP_ADDRESS) & 0xff;
  } else {
    pdw1000local->tempP = 0;
  }

  // Load leading edge detect code (LDE/microcode)
  if (!(DWT_DW_WAKE_UP & config)) {
    if (DWT_LOADUCODE & config) {
      _dwt_loaducodefromrom();
      pdw1000local->sleep_mode |= AON_WCFG_ONW_LLDE; // microcode must be loaded at wake-up if loaded on initialisation
    } else                                           // Should disable the LDERUN bit enable if LDE has not been loaded
    {
      uint16_t rega = dwt_read16bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET + 1);
      rega &= 0xFDFF; // Clear LDERUN bit
      dwt_write16bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET + 1, rega);
    }
  } else // if DWT_DW_WUP_NO_UCODE is set then assume that the UCODE was loaded from ROM (i.e. DWT_LOADUCODE was set on power up),
  {      // thus set AON_WCFG_ONW_LLDE, otherwise don't set the AON_WCFG_ONW_LLDE bit in the sleep_mode configuration
    if ((DWT_DW_WUP_NO_UCODE & config) == 0) {
      pdw1000local->sleep_mode |= AON_WCFG_ONW_LLDE;
    }
  }

  _dwt_enableclocks(ENABLE_ALL_SEQ); // Enable clocks for sequencing

  // The 3 bits in AON CFG1 register must be cleared to ensure proper operation of the DW1000 in DEEPSLEEP mode.
  dwt_write8bitoffsetreg(AON_ID, AON_CFG1_OFFSET, 0x00);

  // Read system register / store local copy
  pdw1000local->sysCFGreg  = dwt_read32bitreg(SYS_CFG_ID);                                             // Read sysconfig register
  pdw1000local->longFrames = (pdw1000local->sysCFGreg & SYS_CFG_PHR_MODE_11) >> SYS_CFG_PHR_MODE_SHFT; // configure longFrames

  pdw1000local->txFCTRL = dwt_read32bitreg(TX_FCTRL_ID);

  return DWT_SUCCESS;

} // end dwt_initialise()

uint8_t dwt_otprevision(void) {
  return pdw1000local->otprev;
}

void dwt_setfinegraintxseq(int enable) {
  if (enable) {
    dwt_write16bitoffsetreg(PMSC_ID, PMSC_TXFINESEQ_OFFSET, PMSC_TXFINESEQ_ENABLE);
  } else {
    dwt_write16bitoffsetreg(PMSC_ID, PMSC_TXFINESEQ_OFFSET, PMSC_TXFINESEQ_DISABLE);
  }
}

void dwt_setlnapamode(int lna_pa) {
  uint32_t gpio_mode = dwt_read32bitoffsetreg(GPIO_CTRL_ID, GPIO_MODE_OFFSET);
  gpio_mode &= ~(GPIO_MSGP4_MASK | GPIO_MSGP5_MASK | GPIO_MSGP6_MASK);
  if (lna_pa & DWT_LNA_ENABLE) {
    gpio_mode |= GPIO_PIN6_EXTRXE;
  }
  if (lna_pa & DWT_PA_ENABLE) {
    gpio_mode |= (GPIO_PIN5_EXTTXE | GPIO_PIN4_EXTPA);
  }
  dwt_write32bitoffsetreg(GPIO_CTRL_ID, GPIO_MODE_OFFSET, gpio_mode);
}

void dwt_enablegpioclocks(void) {
  uint32_t pmsc_clock_ctrl = dwt_read32bitreg(PMSC_ID);
  dwt_write32bitreg(PMSC_ID, pmsc_clock_ctrl | PMSC_CTRL0_GPCE | PMSC_CTRL0_GPRN);
}

void dwt_setgpiodirection(uint32_t gpioNum, uint32_t direction) {
  uint8_t buf[GPIO_DIR_LEN];
  uint32_t command = direction | gpioNum;

  buf[0] = command & 0xff;
  buf[1] = (command >> 8) & 0xff;
  buf[2] = (command >> 16) & 0xff;

  dwt_writetodevice(GPIO_CTRL_ID, GPIO_DIR_OFFSET, GPIO_DIR_LEN, buf);
}

void dwt_setgpiovalue(uint32_t gpioNum, uint32_t value) {
  uint8_t buf[GPIO_DOUT_LEN];
  uint32_t command = value | gpioNum;

  buf[0] = command & 0xff;
  buf[1] = (command >> 8) & 0xff;
  buf[2] = (command >> 16) & 0xff;

  dwt_writetodevice(GPIO_CTRL_ID, GPIO_DOUT_OFFSET, GPIO_DOUT_LEN, buf);
}

int dwt_getgpiovalue(uint32_t gpioNum) {
  return ((dwt_read32bitoffsetreg(GPIO_CTRL_ID, GPIO_RAW_OFFSET) & gpioNum) ? 1 : 0);
}

uint8_t dwt_geticrefvolt(void) {
#ifdef DWT_API_ERROR_CHECK
  assert(pdw1000local->otp_mask & DWT_READ_OTP_BAT);
#endif
  return pdw1000local->vBatP;
}

uint8_t dwt_geticreftemp(void) {
#ifdef DWT_API_ERROR_CHECK
  assert(pdw1000local->otp_mask & DWT_READ_OTP_TMP);
#endif
  return pdw1000local->tempP;
}

uint32_t dwt_getpartid(void) {
#ifdef DWT_API_ERROR_CHECK
  assert(pdw1000local->otp_mask & DWT_READ_OTP_PID);
#endif

  return pdw1000local->partID;
}

uint32_t dwt_getlotid(void) {
#ifdef DWT_API_ERROR_CHECK
  assert(pdw1000local->otp_mask & DWT_READ_OTP_LID);
#endif

  return pdw1000local->lotID;
}

uint32_t dwt_readdevid(void) {
  return dwt_read32bitoffsetreg(DEV_ID_ID, 0);
}

void dwt_configuretxrf(dwt_txconfig_t* config) {

  // Configure RF TX PG_DELAY
  dwt_write8bitoffsetreg(TX_CAL_ID, TC_PGDELAY_OFFSET, config->PGdly);

  // Configure TX power
  dwt_write32bitreg(TX_POWER_ID, config->power);
}

void dwt_configurefor64plen(int prf) {
  dwt_write8bitoffsetreg(CRTR_ID, CRTR_GEAR_OFFSET, DEMOD_GEAR_64L);

  if (prf == DWT_PRF_16M) {
    dwt_write8bitoffsetreg(DRX_CONF_ID, DRX_TUNE2_OFFSET + 2, DRX_TUNE2_UNCONF_SFD_TH_PRF16);
  } else {
    dwt_write8bitoffsetreg(DRX_CONF_ID, DRX_TUNE2_OFFSET + 2, DRX_TUNE2_UNCONF_SFD_TH_PRF64);
  }
}

void dwt_configure(dwt_config_t* config) {
  uint8_t nsSfd_result = 0;
  uint8_t useDWnsSFD   = 0;
  uint8_t chan         = config->chan;
  uint32_t regval;
  uint16_t reg16   = lde_replicaCoeff[config->rxCode];
  uint8_t prfIndex = config->prf - DWT_PRF_16M;
  uint8_t bw       = ((chan == 4) || (chan == 7)) ? 1 : 0; // Select wide or narrow band

#ifdef DWT_API_ERROR_CHECK
  assert(config->dataRate <= DWT_BR_6M8);
  assert(config->rxPAC <= DWT_PAC64);
  assert((chan >= 1) && (chan <= 7) && (chan != 6));
  assert(((config->prf == DWT_PRF_64M) && (config->txCode >= 9) && (config->txCode <= 24))
         || ((config->prf == DWT_PRF_16M) && (config->txCode >= 1) && (config->txCode <= 8)));
  assert(((config->prf == DWT_PRF_64M) && (config->rxCode >= 9) && (config->rxCode <= 24))
         || ((config->prf == DWT_PRF_16M) && (config->rxCode >= 1) && (config->rxCode <= 8)));
  assert((config->txPreambLength == DWT_PLEN_64) || (config->txPreambLength == DWT_PLEN_128) || (config->txPreambLength == DWT_PLEN_256)
         || (config->txPreambLength == DWT_PLEN_512) || (config->txPreambLength == DWT_PLEN_1024) || (config->txPreambLength == DWT_PLEN_1536)
         || (config->txPreambLength == DWT_PLEN_2048) || (config->txPreambLength == DWT_PLEN_4096));
  assert((config->phrMode == DWT_PHRMODE_STD) || (config->phrMode == DWT_PHRMODE_EXT));
#endif

  // For 110 kbps we need a special setup
  if (DWT_BR_110K == config->dataRate) {
    pdw1000local->sysCFGreg |= SYS_CFG_RXM110K;
    reg16 >>= 3; // lde_replicaCoeff must be divided by 8
  } else {
    pdw1000local->sysCFGreg &= (~SYS_CFG_RXM110K);
  }

  pdw1000local->longFrames = config->phrMode;

  pdw1000local->sysCFGreg &= ~SYS_CFG_PHR_MODE_11;
  pdw1000local->sysCFGreg |= (SYS_CFG_PHR_MODE_11 & ((uint32_t)config->phrMode << SYS_CFG_PHR_MODE_SHFT));

  dwt_write32bitreg(SYS_CFG_ID, pdw1000local->sysCFGreg);
  // Set the lde_replicaCoeff
  dwt_write16bitoffsetreg(LDE_IF_ID, LDE_REPC_OFFSET, reg16);

  _dwt_configlde(prfIndex);

  // Configure PLL2/RF PLL block CFG/TUNE (for a given channel)
  dwt_write32bitoffsetreg(FS_CTRL_ID, FS_PLLCFG_OFFSET, fs_pll_cfg[chan_idx[chan]]);
  dwt_write8bitoffsetreg(FS_CTRL_ID, FS_PLLTUNE_OFFSET, fs_pll_tune[chan_idx[chan]]);

  // Configure RF RX blocks (for specified channel/bandwidth)
  dwt_write8bitoffsetreg(RF_CONF_ID, RF_RXCTRLH_OFFSET, rx_config[bw]);

  // Configure RF TX blocks (for specified channel and PRF)
  // Configure RF TX control
  dwt_write32bitoffsetreg(RF_CONF_ID, RF_TXCTRL_OFFSET, tx_config[chan_idx[chan]]);

  // Configure the baseband parameters (for specified PRF, bit rate, PAC, and SFD settings)
  // DTUNE0
  dwt_write16bitoffsetreg(DRX_CONF_ID, DRX_TUNE0b_OFFSET, sftsh[config->dataRate][config->nsSFD]);

  // DTUNE1
  dwt_write16bitoffsetreg(DRX_CONF_ID, DRX_TUNE1a_OFFSET, dtune1[prfIndex]);

  if (config->dataRate == DWT_BR_110K) {
    dwt_write16bitoffsetreg(DRX_CONF_ID, DRX_TUNE1b_OFFSET, DRX_TUNE1b_110K);
  } else {
    if (config->txPreambLength == DWT_PLEN_64) {
      dwt_write16bitoffsetreg(DRX_CONF_ID, DRX_TUNE1b_OFFSET, DRX_TUNE1b_6M8_PRE64);
      dwt_write8bitoffsetreg(DRX_CONF_ID, DRX_TUNE4H_OFFSET, DRX_TUNE4H_PRE64);
    } else {
      dwt_write16bitoffsetreg(DRX_CONF_ID, DRX_TUNE1b_OFFSET, DRX_TUNE1b_850K_6M8);
      dwt_write8bitoffsetreg(DRX_CONF_ID, DRX_TUNE4H_OFFSET, DRX_TUNE4H_PRE128PLUS);
    }
  }

  // DTUNE2
  dwt_write32bitoffsetreg(DRX_CONF_ID, DRX_TUNE2_OFFSET, digital_bb_config[prfIndex][config->rxPAC]);

  // DTUNE3 (SFD timeout)
  // Don't allow 0 - SFD timeout will always be enabled
  if (config->sfdTO == 0) {
    config->sfdTO = DWT_SFDTOC_DEF;
  }
  dwt_write16bitoffsetreg(DRX_CONF_ID, DRX_SFDTOC_OFFSET, config->sfdTO);

  // Configure AGC parameters
  dwt_write32bitoffsetreg(AGC_CFG_STS_ID, 0xC, agc_config.lo32);
  dwt_write16bitoffsetreg(AGC_CFG_STS_ID, 0x4, agc_config.target[prfIndex]);

  // Set (non-standard) user SFD for improved performance,
  if (config->nsSFD) {
    // Write non standard (DW) SFD length
    dwt_write8bitoffsetreg(USR_SFD_ID, 0x00, dwnsSFDlen[config->dataRate]);
    nsSfd_result = 3;
    useDWnsSFD   = 1;
  }
  regval = (CHAN_CTRL_TX_CHAN_MASK & (chan << CHAN_CTRL_TX_CHAN_SHIFT)) |                                 // Transmit Channel
           (CHAN_CTRL_RX_CHAN_MASK & (chan << CHAN_CTRL_RX_CHAN_SHIFT)) |                                 // Receive Channel
           (CHAN_CTRL_RXFPRF_MASK & ((uint32_t)config->prf << CHAN_CTRL_RXFPRF_SHIFT)) |                  // RX PRF
           ((CHAN_CTRL_TNSSFD | CHAN_CTRL_RNSSFD) & ((uint32_t)nsSfd_result << CHAN_CTRL_TNSSFD_SHIFT)) | // nsSFD enable RX&TX
           (CHAN_CTRL_DWSFD & ((uint32_t)useDWnsSFD << CHAN_CTRL_DWSFD_SHIFT)) |                          // Use DW nsSFD
           (CHAN_CTRL_TX_PCOD_MASK & ((uint32_t)config->txCode << CHAN_CTRL_TX_PCOD_SHIFT)) |             // TX Preamble Code
           (CHAN_CTRL_RX_PCOD_MASK & ((uint32_t)config->rxCode << CHAN_CTRL_RX_PCOD_SHIFT));              // RX Preamble Code

  dwt_write32bitreg(CHAN_CTRL_ID, regval);

  // Set up TX Preamble Size, PRF and Data Rate
  pdw1000local->txFCTRL = ((uint32_t)(config->txPreambLength | config->prf) << TX_FCTRL_TXPRF_SHFT) | ((uint32_t)config->dataRate << TX_FCTRL_TXBR_SHFT);
  dwt_write32bitreg(TX_FCTRL_ID, pdw1000local->txFCTRL);

  // The SFD transmit pattern is initialised by the DW1000 upon a user TX request, but (due to an IC issue) it is not done for an auto-ACK TX. The
  // SYS_CTRL write below works around this issue, by simultaneously initiating and aborting a transmission, which correctly initialises the SFD
  // after its configuration or reconfiguration.
  // This issue is not documented at the time of writing this code. It should be in next release of DW1000 User Manual (v2.09, from July 2016).
  dwt_write8bitoffsetreg(SYS_CTRL_ID, SYS_CTRL_OFFSET, SYS_CTRL_TXSTRT | SYS_CTRL_TRXOFF); // Request TX start and TRX off at the same time
} // end dwt_configure()

void dwt_setrxantennadelay(uint16_t rxDelay) {
  // Set the RX antenna delay for auto TX timestamp adjustment
  dwt_write16bitoffsetreg(LDE_IF_ID, LDE_RXANTD_OFFSET, rxDelay);
}

void dwt_settxantennadelay(uint16_t txDelay) {
  // Set the TX antenna delay for auto TX timestamp adjustment
  dwt_write16bitoffsetreg(TX_ANTD_ID, TX_ANTD_OFFSET, txDelay);
}

int dwt_writetxdata(uint16_t txFrameLength, uint8_t* txFrameBytes, uint16_t txBufferOffset) {
#ifdef DWT_API_ERROR_CHECK
  assert(txFrameLength >= 2);
  assert((pdw1000local->longFrames && (txFrameLength <= 1023)) || (txFrameLength <= 127));
  assert((txBufferOffset + txFrameLength) <= 1024);
#endif

  if ((txBufferOffset + txFrameLength) <= 1024) {
    // Write the data to the IC TX buffer, (-2 bytes for auto generated CRC)
    dwt_writetodevice(TX_BUFFER_ID, txBufferOffset, txFrameLength - 2, txFrameBytes);
    return DWT_SUCCESS;
  } else {
    return DWT_ERROR;
  }
} // end dwt_writetxdata()

void dwt_writetxfctrl(uint16_t txFrameLength, uint16_t txBufferOffset, int ranging) {

#ifdef DWT_API_ERROR_CHECK
  assert((pdw1000local->longFrames && (txFrameLength <= 1023)) || (txFrameLength <= 127));
  assert((txBufferOffset + txFrameLength) <= 1024);
  assert((ranging == 0) || (ranging == 1))
#endif

      // Write the frame length to the TX frame control register
      // pdw1000local->txFCTRL has kept configured bit rate information
      uint32_t reg32
      = pdw1000local->txFCTRL | txFrameLength | ((uint32_t)txBufferOffset << TX_FCTRL_TXBOFFS_SHFT) | ((uint32_t)ranging << TX_FCTRL_TR_SHFT);
  dwt_write32bitreg(TX_FCTRL_ID, reg32);
} // end dwt_writetxfctrl()

void dwt_readrxdata(uint8_t* buffer, uint16_t length, uint16_t rxBufferOffset) {
  dwt_readfromdevice(RX_BUFFER_ID, rxBufferOffset, length, buffer);
}

void dwt_readaccdata(uint8_t* buffer, uint16_t len, uint16_t accOffset) {
  // Force on the ACC clocks if we are sequenced
  _dwt_enableclocks(READ_ACC_ON);

  dwt_readfromdevice(ACC_MEM_ID, accOffset, len, buffer);

  _dwt_enableclocks(READ_ACC_OFF); // Revert clocks back
}

#define B20_SIGN_EXTEND_TEST (0x00100000UL)
#define B20_SIGN_EXTEND_MASK (0xFFF00000UL)

int32_t dwt_readcarrierintegrator(void) {
  uint32_t regval = 0;
  int j;
  uint8_t buffer[DRX_CARRIER_INT_LEN];

  /* Read 3 bytes into buffer (21-bit quantity) */

  dwt_readfromdevice(DRX_CONF_ID, DRX_CARRIER_INT_OFFSET, DRX_CARRIER_INT_LEN, buffer);

  for (j = 2; j >= 0; j--) // arrange the three bytes into an unsigned integer value
  {
    regval = (regval << 8) + buffer[j];
  }

  if (regval & B20_SIGN_EXTEND_TEST)
    regval |= B20_SIGN_EXTEND_MASK; // sign extend bit #20 to whole word
  else
    regval &= DRX_CARRIER_INT_MASK; // make sure upper bits are clear if not sign extending

  return (int32_t)regval;           // cast unsigned value to signed quantity.
}

void dwt_readdiagnostics(dwt_rxdiag_t* diagnostics) {
  // Read the HW FP index
  diagnostics->firstPath = dwt_read16bitoffsetreg(RX_TIME_ID, RX_TIME_FP_INDEX_OFFSET);

  // LDE diagnostic data
  diagnostics->maxNoise = dwt_read16bitoffsetreg(LDE_IF_ID, LDE_THRESH_OFFSET);

  // Read all 8 bytes in one SPI transaction
  dwt_readfromdevice(RX_FQUAL_ID, 0x0, 8, (uint8_t*)&diagnostics->stdNoise);

  diagnostics->firstPathAmp1 = dwt_read16bitoffsetreg(RX_TIME_ID, RX_TIME_FP_AMPL1_OFFSET);

  diagnostics->rxPreamCount = (dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXPACC_MASK) >> RX_FINFO_RXPACC_SHIFT;
}

void dwt_readtxtimestamp(uint8_t* timestamp) {
  dwt_readfromdevice(TX_TIME_ID, TX_TIME_TX_STAMP_OFFSET, TX_TIME_TX_STAMP_LEN, timestamp); // Read bytes directly into buffer
}

uint32_t dwt_readtxtimestamphi32(void) {
  return dwt_read32bitoffsetreg(TX_TIME_ID, 1); // Offset is 1 to get the 4 upper bytes out of 5
}

uint32_t dwt_readtxtimestamplo32(void) {
  return dwt_read32bitreg(TX_TIME_ID); // Read TX TIME as a 32-bit register to get the 4 lower bytes out of 5
}

void dwt_readrxtimestamp(uint8_t* timestamp) {
  dwt_readfromdevice(RX_TIME_ID, RX_TIME_RX_STAMP_OFFSET, RX_TIME_RX_STAMP_LEN, timestamp); // Get the adjusted time of arrival
}

uint32_t dwt_readrxtimestamphi32(void) {
  return dwt_read32bitoffsetreg(RX_TIME_ID, 1); // Offset is 1 to get the 4 upper bytes out of 5
}

uint32_t dwt_readrxtimestamplo32(void) {
  return dwt_read32bitreg(RX_TIME_ID); // Read RX TIME as a 32-bit register to get the 4 lower bytes out of 5
}

uint32_t dwt_readsystimestamphi32(void) {
  return dwt_read32bitoffsetreg(SYS_TIME_ID, 1); // Offset is 1 to get the 4 upper bytes out of 5
}

void dwt_readsystime(uint8_t* timestamp) {
  dwt_readfromdevice(SYS_TIME_ID, SYS_TIME_OFFSET, SYS_TIME_LEN, timestamp);
}

void dwt_writetodevice(uint16_t recordNumber, uint16_t index, uint32_t length, const uint8_t* buffer) {
  uint8_t header[3];            // Buffer to compose header in
  int cnt = 0;                  // Counter for length of header
#ifdef DWT_API_ERROR_CHECK
  assert(recordNumber <= 0x3F); // Record number is limited to 6-bits.
#endif

  // Write message header selecting WRITE operation and addresses as appropriate (this is one to three bytes long)
  if (index == 0)                        // For index of 0, no sub-index is required
  {
    header[cnt++] = 0x80 | recordNumber; // Bit-7 is WRITE operation, bit-6 zero=NO sub-addressing, bits 5-0 is reg file id
  } else {
#ifdef DWT_API_ERROR_CHECK
    assert((index <= 0x7FFF) && ((index + length) <= 0x7FFF)); // Index and sub-addressable area are limited to 15-bits.
#endif
    header[cnt++] = 0xC0 | recordNumber;                       // Bit-7 is WRITE operation, bit-6 one=sub-address follows, bits 5-0 is reg file id

    if (index <= 127)                                          // For non-zero index < 127, just a single sub-index byte is required
    {
      header[cnt++] = (uint8_t)index;                          // Bit-7 zero means no extension, bits 6-0 is index.
    } else {
      header[cnt++] = 0x80 | (uint8_t)(index);                 // Bit-7 one means extended index, bits 6-0 is low seven bits of index.
      header[cnt++] = (uint8_t)(index >> 7);                   // 8-bit value = high eight bits of index.
    }
  }

  // Write it to the SPI
  writetospi(cnt, header, length, buffer);
} // end dwt_writetodevice()

void dwt_readfromdevice(uint16_t recordNumber, uint16_t index, uint32_t length, uint8_t* buffer) {
  uint8_t header[3];            // Buffer to compose header in
  int cnt = 0;                  // Counter for length of header
#ifdef DWT_API_ERROR_CHECK
  assert(recordNumber <= 0x3F); // Record number is limited to 6-bits.
#endif

  // Write message header selecting READ operation and addresses as appropriate (this is one to three bytes long)
  if (index == 0)                          // For index of 0, no sub-index is required
  {
    header[cnt++] = (uint8_t)recordNumber; // Bit-7 zero is READ operation, bit-6 zero=NO sub-addressing, bits 5-0 is reg file id
  } else {
#ifdef DWT_API_ERROR_CHECK
    assert((index <= 0x7FFF) && ((index + length) <= 0x7FFF)); // Index and sub-addressable area are limited to 15-bits.
#endif
    header[cnt++] = (uint8_t)(0x40 | recordNumber);            // Bit-7 zero is READ operation, bit-6 one=sub-address follows, bits 5-0 is reg file id

    if (index <= 127)                                          // For non-zero index < 127, just a single sub-index byte is required
    {
      header[cnt++] = (uint8_t)index;                          // Bit-7 zero means no extension, bits 6-0 is index.
    } else {
      header[cnt++] = 0x80 | (uint8_t)(index);                 // Bit-7 one means extended index, bits 6-0 is low seven bits of index.
      header[cnt++] = (uint8_t)(index >> 7);                   // 8-bit value = high eight bits of index.
    }
  }

  // Do the read from the SPI
  readfromspi(cnt, header, length, buffer); // result is stored in the buffer
} // end dwt_readfromdevice()

uint32_t dwt_read32bitoffsetreg(int regFileID, int regOffset) {
  uint32_t regval = 0;
  int j;
  uint8_t buffer[4];

  dwt_readfromdevice(regFileID, regOffset, 4, buffer); // Read 4 bytes (32-bits) register into buffer

  for (j = 3; j >= 0; j--) {
    regval = (regval << 8) + buffer[j];
  }
  return regval;

} // end dwt_read32bitoffsetreg()

uint16_t dwt_read16bitoffsetreg(int regFileID, int regOffset) {
  uint16_t regval = 0;
  uint8_t buffer[2];

  dwt_readfromdevice(regFileID, regOffset, 2, buffer); // Read 2 bytes (16-bits) register into buffer

  regval = ((uint16_t)buffer[1] << 8) + buffer[0];
  return regval;

} // end dwt_read16bitoffsetreg()

uint8_t dwt_read8bitoffsetreg(int regFileID, int regOffset) {
  uint8_t regval;

  dwt_readfromdevice(regFileID, regOffset, 1, &regval);

  return regval;
}

void dwt_write8bitoffsetreg(int regFileID, int regOffset, uint8_t regval) {
  dwt_writetodevice(regFileID, regOffset, 1, &regval);
}

void dwt_write16bitoffsetreg(int regFileID, int regOffset, uint16_t regval) {
  uint8_t buffer[2];

  buffer[0] = regval & 0xFF;
  buffer[1] = regval >> 8;

  dwt_writetodevice(regFileID, regOffset, 2, buffer);
} // end dwt_write16bitoffsetreg()

void dwt_write32bitoffsetreg(int regFileID, int regOffset, uint32_t regval) {
  int j;
  uint8_t buffer[4];

  for (j = 0; j < 4; j++) {
    buffer[j] = regval & 0xff;
    regval >>= 8;
  }

  dwt_writetodevice(regFileID, regOffset, 4, buffer);
} // end dwt_write32bitoffsetreg()

void dwt_enableframefilter(uint16_t enable) {
  uint32_t sysconfig = SYS_CFG_MASK & dwt_read32bitreg(SYS_CFG_ID); // Read sysconfig register

  if (enable) {
    // Enable frame filtering and configure frame types
    sysconfig &= ~(SYS_CFG_FF_ALL_EN); // Clear all
    sysconfig |= (enable & SYS_CFG_FF_ALL_EN) | SYS_CFG_FFE;
  } else {
    sysconfig &= ~(SYS_CFG_FFE);
  }

  pdw1000local->sysCFGreg = sysconfig;
  dwt_write32bitreg(SYS_CFG_ID, sysconfig);
}

void dwt_setpanid(uint16_t panID) {
  // PAN ID is high 16 bits of register
  dwt_write16bitoffsetreg(PANADR_ID, PANADR_PAN_ID_OFFSET, panID);
}

void dwt_setaddress16(uint16_t shortAddress) {
  // Short address into low 16 bits
  dwt_write16bitoffsetreg(PANADR_ID, PANADR_SHORT_ADDR_OFFSET, shortAddress);
}

void dwt_seteui(uint8_t* eui64) {
  dwt_writetodevice(EUI_64_ID, EUI_64_OFFSET, EUI_64_LEN, eui64);
}

void dwt_geteui(uint8_t* eui64) {
  dwt_readfromdevice(EUI_64_ID, EUI_64_OFFSET, EUI_64_LEN, eui64);
}

void dwt_otpread(uint16_t address, uint32_t* array, uint8_t length) {
  int i;

  _dwt_enableclocks(FORCE_SYS_XTI); // NOTE: Set system clock to XTAL - this is necessary to make sure the values read by _dwt_otpread are reliable

  for (i = 0; i < length; i++) {
    array[i] = _dwt_otpread(address + i);
  }

  _dwt_enableclocks(ENABLE_ALL_SEQ); // Restore system clock to PLL

  return;
}

uint32_t _dwt_otpread(uint16_t address) {
  uint32_t ret_data;

  // Write the address
  dwt_write16bitoffsetreg(OTP_IF_ID, OTP_ADDR, address);

  // Perform OTP Read - Manual read mode has to be set
  dwt_write8bitoffsetreg(OTP_IF_ID, OTP_CTRL, OTP_CTRL_OTPREAD | OTP_CTRL_OTPRDEN);
  dwt_write8bitoffsetreg(OTP_IF_ID, OTP_CTRL, 0x00); // OTPREAD is self clearing but OTPRDEN is not

  // Read read data, available 40ns after rising edge of OTP_READ
  ret_data = dwt_read32bitoffsetreg(OTP_IF_ID, OTP_RDAT);

  // Return the 32bit of read data
  return ret_data;
}

uint32_t _dwt_otpsetmrregs(int mode) {
  uint8_t wr_buf[4];
  uint32_t mra = 0, mrb = 0, mr = 0;

  // PROGRAMME MRA
  // Set MRA, MODE_SEL
  wr_buf[0] = 0x03;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL + 1, 1, wr_buf);

  // Load data
  switch (mode & 0x0f) {
  case 0x0:
    mr  = 0x0000;
    mra = 0x0000;
    mrb = 0x0000;
    break;
  case 0x1:
    mr  = 0x1024;
    mra = 0x9220; // Enable CPP mon
    mrb = 0x000e;
    break;
  case 0x2:
    mr  = 0x1824;
    mra = 0x9220;
    mrb = 0x0003;
    break;
  case 0x3:
    mr  = 0x1824;
    mra = 0x9220;
    mrb = 0x004e;
    break;
  case 0x4:
    mr  = 0x0000;
    mra = 0x0000;
    mrb = 0x0003;
    break;
  case 0x5:
    mr  = 0x0024;
    mra = 0x0000;
    mrb = 0x0003;
    break;
  default:
    return DWT_ERROR;
  }

  wr_buf[0] = mra & 0x00ff;
  wr_buf[1] = (mra & 0xff00) >> 8;
  dwt_writetodevice(OTP_IF_ID, OTP_WDAT, 2, wr_buf);

  // Set WRITE_MR
  wr_buf[0] = 0x08;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL, 1, wr_buf);

  // Wait?
  deca_sleep(2);

  // Set Clear Mode sel
  wr_buf[0] = 0x02;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL + 1, 1, wr_buf);

  // Set AUX update, write MR
  wr_buf[0] = 0x88;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL, 1, wr_buf);
  // Clear write MR
  wr_buf[0] = 0x80;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL, 1, wr_buf);
  // Clear AUX update
  wr_buf[0] = 0x00;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL, 1, wr_buf);

  ///////////////////////////////////////////
  // PROGRAM MRB
  // Set SLOW, MRB, MODE_SEL
  wr_buf[0] = 0x05;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL + 1, 1, wr_buf);

  wr_buf[0] = mrb & 0x00ff;
  wr_buf[1] = (mrb & 0xff00) >> 8;
  dwt_writetodevice(OTP_IF_ID, OTP_WDAT, 2, wr_buf);

  // Set WRITE_MR
  wr_buf[0] = 0x08;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL, 1, wr_buf);

  // Wait?
  deca_sleep(2);

  // Set Clear Mode sel
  wr_buf[0] = 0x04;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL + 1, 1, wr_buf);

  // Set AUX update, write MR
  wr_buf[0] = 0x88;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL, 1, wr_buf);
  // Clear write MR
  wr_buf[0] = 0x80;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL, 1, wr_buf);
  // Clear AUX update
  wr_buf[0] = 0x00;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL, 1, wr_buf);

  ///////////////////////////////////////////
  // PROGRAM MR
  // Set SLOW, MODE_SEL
  wr_buf[0] = 0x01;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL + 1, 1, wr_buf);
  // Load data

  wr_buf[0] = mr & 0x00ff;
  wr_buf[1] = (mr & 0xff00) >> 8;
  dwt_writetodevice(OTP_IF_ID, OTP_WDAT, 2, wr_buf);

  // Set WRITE_MR
  wr_buf[0] = 0x08;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL, 1, wr_buf);

  // Wait?
  deca_sleep(2);
  // Set Clear Mode sel
  wr_buf[0] = 0x00;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL + 1, 1, wr_buf);

  return DWT_SUCCESS;
}

uint32_t _dwt_otpprogword32(uint32_t data, uint16_t address) {
  uint8_t rd_buf[1];
  uint8_t wr_buf[4];
  uint8_t otp_done;

  // Write the data
  wr_buf[3] = (data >> 24) & 0xff;
  wr_buf[2] = (data >> 16) & 0xff;
  wr_buf[1] = (data >> 8) & 0xff;
  wr_buf[0] = data & 0xff;
  dwt_writetodevice(OTP_IF_ID, OTP_WDAT, 4, wr_buf);

  // Write the address [10:0]
  wr_buf[1] = (address >> 8) & 0x07;
  wr_buf[0] = address & 0xff;
  dwt_writetodevice(OTP_IF_ID, OTP_ADDR, 2, wr_buf);

  // Enable Sequenced programming
  wr_buf[0] = OTP_CTRL_OTPPROG;
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL, 1, wr_buf);
  wr_buf[0] = 0x00; // And clear
  dwt_writetodevice(OTP_IF_ID, OTP_CTRL, 1, wr_buf);

  // WAIT for status to flag PRGM OK..
  otp_done = 0;
  while (otp_done == 0) {
    deca_sleep(1);
    dwt_readfromdevice(OTP_IF_ID, OTP_STAT, 1, rd_buf);

    if ((rd_buf[0] & 0x01) == 0x01) {
      otp_done = 1;
    }
  }

  return DWT_SUCCESS;
}

int dwt_otpwriteandverify(uint32_t value, uint16_t address) {
  int prog_ok = DWT_SUCCESS;
  int retry   = 0;
  // Firstly set the system clock to crystal
  _dwt_enableclocks(FORCE_SYS_XTI); // set system clock to XTI

  //
  //!!!!!!!!!!!!!! NOTE !!!!!!!!!!!!!!!!!!!!!
  // Set the supply to 3.7V
  //

  _dwt_otpsetmrregs(1); // Set mode for programming

  // For each value to program - the readback/check is done couple of times to verify it has programmed successfully
  while (1) {
    _dwt_otpprogword32(value, address);

    if (_dwt_otpread(address) == value) {
      break;
    }
    retry++;
    if (retry == 10) {
      break;
    }
  }

  // Even if the above does not exit before retry reaches 10, the programming has probably been successful

  _dwt_otpsetmrregs(4);               // Set mode for reading

  if (_dwt_otpread(address) != value) // If this does not pass please check voltage supply on VDDIO
  {
    prog_ok = DWT_ERROR;
  }

  _dwt_otpsetmrregs(0); // Setting OTP mode register for low RM read - resetting the device would be alternative

  return prog_ok;
}

void _dwt_aonconfigupload(void) {
  dwt_write8bitoffsetreg(AON_ID, AON_CTRL_OFFSET, AON_CTRL_UPL_CFG);
  dwt_write8bitoffsetreg(AON_ID, AON_CTRL_OFFSET, 0x00); // Clear the register
}

void _dwt_aonarrayupload(void) {
  dwt_write8bitoffsetreg(AON_ID, AON_CTRL_OFFSET, 0x00); // Clear the register
  dwt_write8bitoffsetreg(AON_ID, AON_CTRL_OFFSET, AON_CTRL_SAVE);
}

void dwt_entersleep(void) {
  // Copy config to AON - upload the new configuration
  _dwt_aonarrayupload();
}

void dwt_configuresleepcnt(uint16_t sleepcnt) {
  // Force system clock to crystal
  _dwt_enableclocks(FORCE_SYS_XTI);

  // Reset sleep configuration to make sure we don't accidentally go to sleep
  dwt_write8bitoffsetreg(AON_ID, AON_CFG0_OFFSET, 0x00); // NB: this write change the default LPCLKDIVA value which is not used anyway.
  dwt_write8bitoffsetreg(AON_ID, AON_CFG1_OFFSET, 0x00);

  // Disable the sleep counter
  _dwt_aonconfigupload();

  // Set new value
  dwt_write16bitoffsetreg(AON_ID, AON_CFG0_OFFSET + AON_CFG0_SLEEP_TIM_OFFSET, sleepcnt);
  _dwt_aonconfigupload();

  // Enable the sleep counter
  dwt_write8bitoffsetreg(AON_ID, AON_CFG1_OFFSET, AON_CFG1_SLEEP_CEN);
  _dwt_aonconfigupload();

  // Put system PLL back on
  _dwt_enableclocks(ENABLE_ALL_SEQ);
}

uint16_t dwt_calibratesleepcnt(void) {
  uint16_t result;

  // Enable calibration of the sleep counter
  dwt_write8bitoffsetreg(AON_ID, AON_CFG1_OFFSET, AON_CFG1_LPOSC_CAL);
  _dwt_aonconfigupload();

  // Disable calibration of the sleep counter
  dwt_write8bitoffsetreg(AON_ID, AON_CFG1_OFFSET, 0x00);
  _dwt_aonconfigupload();

  // Force system clock to crystal
  _dwt_enableclocks(FORCE_SYS_XTI);

  deca_sleep(1);

  // Read the number of XTAL/2 cycles one LP oscillator cycle took.
  // Set up address - Read upper byte first
  dwt_write8bitoffsetreg(AON_ID, AON_ADDR_OFFSET, AON_ADDR_LPOSC_CAL_1);

  // Enable manual override
  dwt_write8bitoffsetreg(AON_ID, AON_CTRL_OFFSET, AON_CTRL_DCA_ENAB);

  // Read confirm data that was written
  dwt_write8bitoffsetreg(AON_ID, AON_CTRL_OFFSET, AON_CTRL_DCA_ENAB | AON_CTRL_DCA_READ);

  // Read back byte from AON
  result = dwt_read8bitoffsetreg(AON_ID, AON_RDAT_OFFSET);
  result <<= 8;

  // Set up address - Read lower byte
  dwt_write8bitoffsetreg(AON_ID, AON_ADDR_OFFSET, AON_ADDR_LPOSC_CAL_0);

  // Enable manual override
  dwt_write8bitoffsetreg(AON_ID, AON_CTRL_OFFSET, AON_CTRL_DCA_ENAB);

  // Read confirm data that was written
  dwt_write8bitoffsetreg(AON_ID, AON_CTRL_OFFSET, AON_CTRL_DCA_ENAB | AON_CTRL_DCA_READ);

  // Read back byte from AON
  result |= dwt_read8bitoffsetreg(AON_ID, AON_RDAT_OFFSET);

  // Disable manual override
  dwt_write8bitoffsetreg(AON_ID, AON_CTRL_OFFSET, 0x00);

  // Put system PLL back on
  _dwt_enableclocks(ENABLE_ALL_SEQ);

  // Returns the number of XTAL/2 cycles per one LP OSC cycle
  // This can be converted into LP OSC frequency by 19.2 MHz/result
  return result;
}

void dwt_configuresleep(uint16_t mode, uint8_t wake) {
  // Add predefined sleep settings before writing the mode
  mode |= pdw1000local->sleep_mode;
  dwt_write16bitoffsetreg(AON_ID, AON_WCFG_OFFSET, mode);

  dwt_write8bitoffsetreg(AON_ID, AON_CFG0_OFFSET, wake);
}

void dwt_entersleepaftertx(int enable) {
  uint32_t reg = dwt_read32bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET);
  // Set the auto TX -> sleep bit
  if (enable) {
    reg |= PMSC_CTRL1_ATXSLP;
  } else {
    reg &= ~(PMSC_CTRL1_ATXSLP);
  }
  dwt_write32bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET, reg);
}

int dwt_spicswakeup(uint8_t* buff, uint16_t length) {
  if (dwt_readdevid() != DWT_DEVICE_ID) // Device was in deep sleep (the first read fails)
  {
    // Need to keep chip select line low for at least 500us
    dwt_readfromdevice(0x0, 0x0, length, buff); // Do a long read to wake up the chip (hold the chip select low)

    // Need 5ms for XTAL to start and stabilise (could wait for PLL lock IRQ status bit !!!)
    // NOTE: Polling of the STATUS register is not possible unless frequency is < 3MHz
    deca_sleep(5);
  } else {
    return DWT_SUCCESS;
  }
  // DEBUG - check if still in sleep mode
  if (dwt_readdevid() != DWT_DEVICE_ID) {
    return DWT_ERROR;
  }

  return DWT_SUCCESS;
}

void _dwt_configlde(int prf) {
  dwt_write8bitoffsetreg(LDE_IF_ID, LDE_CFG1_OFFSET, LDE_PARAM1); // 8-bit configuration register

  if (prf) {
    dwt_write16bitoffsetreg(LDE_IF_ID, LDE_CFG2_OFFSET, (uint16_t)LDE_PARAM3_64); // 16-bit LDE configuration tuning register
  } else {
    dwt_write16bitoffsetreg(LDE_IF_ID, LDE_CFG2_OFFSET, (uint16_t)LDE_PARAM3_16);
  }
}

void _dwt_loaducodefromrom(void) {
  // Set up clocks
  _dwt_enableclocks(FORCE_LDE);

  // Kick off the LDE load
  dwt_write16bitoffsetreg(OTP_IF_ID, OTP_CTRL, OTP_CTRL_LDELOAD); // Set load LDE kick bit

  deca_sleep(1);                                                  // Allow time for code to upload (should take up to 120 us)

  // Default clocks (ENABLE_ALL_SEQ)
  _dwt_enableclocks(ENABLE_ALL_SEQ); // Enable clocks for sequencing
}

void dwt_loadopsettabfromotp(uint8_t ops_sel) {
  uint16_t reg = ((ops_sel << OTP_SF_OPS_SEL_SHFT) & OTP_SF_OPS_SEL_MASK) | OTP_SF_OPS_KICK; // Select defined OPS table and trigger its loading

  // Set up clocks
  _dwt_enableclocks(FORCE_LDE);

  dwt_write16bitoffsetreg(OTP_IF_ID, OTP_SF, reg);

  // Default clocks (ENABLE_ALL_SEQ)
  _dwt_enableclocks(ENABLE_ALL_SEQ); // Enable clocks for sequencing
}

void dwt_setsmarttxpower(int enable) {
  // Config system register
  pdw1000local->sysCFGreg = dwt_read32bitreg(SYS_CFG_ID); // Read sysconfig register

  // Disable smart power configuration
  if (enable) {
    pdw1000local->sysCFGreg &= ~(SYS_CFG_DIS_STXP);
  } else {
    pdw1000local->sysCFGreg |= SYS_CFG_DIS_STXP;
  }

  dwt_write32bitreg(SYS_CFG_ID, pdw1000local->sysCFGreg);
}

void dwt_enableautoack(uint8_t responseDelayTime) {
  // Set auto ACK reply delay
  dwt_write8bitoffsetreg(ACK_RESP_T_ID, ACK_RESP_T_ACK_TIM_OFFSET, responseDelayTime); // In symbols
  // Enable auto ACK
  pdw1000local->sysCFGreg |= SYS_CFG_AUTOACK;
  dwt_write32bitreg(SYS_CFG_ID, pdw1000local->sysCFGreg);
}

void dwt_setdblrxbuffmode(int enable) {
  if (enable) {
    // Enable double RX buffer mode
    pdw1000local->sysCFGreg &= ~SYS_CFG_DIS_DRXB;
    pdw1000local->dblbuffon = 1;
  } else {
    // Disable double RX buffer mode
    pdw1000local->sysCFGreg |= SYS_CFG_DIS_DRXB;
    pdw1000local->dblbuffon = 0;
  }

  dwt_write32bitreg(SYS_CFG_ID, pdw1000local->sysCFGreg);
}

void dwt_setrxaftertxdelay(uint32_t rxDelayTime) {
  uint32_t val = dwt_read32bitreg(ACK_RESP_T_ID); // Read ACK_RESP_T_ID register

  val &= ~(ACK_RESP_T_W4R_TIM_MASK);              // Clear the timer (19:0)

  val |= (rxDelayTime & ACK_RESP_T_W4R_TIM_MASK); // In UWB microseconds (e.g. turn the receiver on 20uus after TX)

  dwt_write32bitreg(ACK_RESP_T_ID, val);
}

void dwt_setcallbacks(dwt_cb_t cbTxDone, dwt_cb_t cbRxOk, dwt_cb_t cbRxTo, dwt_cb_t cbRxErr) {
  pdw1000local->cbTxDone = cbTxDone;
  pdw1000local->cbRxOk   = cbRxOk;
  pdw1000local->cbRxTo   = cbRxTo;
  pdw1000local->cbRxErr  = cbRxErr;
}

uint8_t dwt_checkirq(void) {
  return (dwt_read8bitoffsetreg(SYS_STATUS_ID, SYS_STATUS_OFFSET) & SYS_STATUS_IRQS); // Reading the lower byte only is enough for this operation
}

void dwt_isr(void) {
  uint32_t status = pdw1000local->cbData.status = dwt_read32bitreg(SYS_STATUS_ID); // Read status register low 32bits

  // Handle RX good frame event
  if (status & SYS_STATUS_RXFCG) {
    uint16_t finfo16;
    uint16_t len;

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_GOOD); // Clear all receive status bits

    pdw1000local->cbData.rx_flags = 0;

    // Read frame info - Only the first two bytes of the register are used here.
    finfo16 = dwt_read16bitoffsetreg(RX_FINFO_ID, RX_FINFO_OFFSET);

    // Report frame length - Standard frame length up to 127, extended frame length up to 1023 bytes
    len = finfo16 & RX_FINFO_RXFL_MASK_1023;
    if (pdw1000local->longFrames == 0) {
      len &= RX_FINFO_RXFLEN_MASK;
    }
    pdw1000local->cbData.datalength = len;

    // Report ranging bit
    if (finfo16 & RX_FINFO_RNG) {
      pdw1000local->cbData.rx_flags |= DWT_CB_DATA_RX_FLAG_RNG;
    }

    // Report frame control - First bytes of the received frame.
    dwt_readfromdevice(RX_BUFFER_ID, 0, FCTRL_LEN_MAX, pdw1000local->cbData.fctrl);

    // Because of a previous frame not being received properly, AAT bit can be set upon the proper reception of a frame not requesting for
    // acknowledgement (ACK frame is not actually sent though). If the AAT bit is set, check ACK request bit in frame control to confirm (this
    // implementation works only for IEEE802.15.4-2011 compliant frames).
    // This issue is not documented at the time of writing this code. It should be in next release of DW1000 User Manual (v2.09, from July 2016).
    if ((status & SYS_STATUS_AAT) && ((pdw1000local->cbData.fctrl[0] & FCTRL_ACK_REQ_MASK) == 0)) {
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_AAT); // Clear AAT status bit in register
      pdw1000local->cbData.status &= ~SYS_STATUS_AAT;   // Clear AAT status bit in callback data register copy
      pdw1000local->wait4resp = 0;
    }

    // Call the corresponding callback if present
    if (pdw1000local->cbRxOk != NULL) {
      pdw1000local->cbRxOk(&pdw1000local->cbData);
    }

    if (pdw1000local->dblbuffon) {
      // Toggle the Host side Receive Buffer Pointer
      dwt_write8bitoffsetreg(SYS_CTRL_ID, SYS_CTRL_HRBT_OFFSET, 1);
    }
  }

  // Handle TX confirmation event
  if (status & SYS_STATUS_TXFRS) {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX); // Clear TX event bits

    // In the case where this TXFRS interrupt is due to the automatic transmission of an ACK solicited by a response (with ACK request bit set)
    // that we receive through using wait4resp to a previous TX (and assuming that the IRQ processing of that TX has already been handled), then
    // we need to handle the IC issue which turns on the RX again in this situation (i.e. because it is wrongly applying the wait4resp after the
    // ACK TX).
    // See section "Transmit and automatically wait for response" in DW1000 User Manual
    if ((status & SYS_STATUS_AAT) && pdw1000local->wait4resp) {
      dwt_forcetrxoff(); // Turn the RX off
      dwt_rxreset();     // Reset in case we were late and a frame was already being received
    }

    // Call the corresponding callback if present
    if (pdw1000local->cbTxDone != NULL) {
      pdw1000local->cbTxDone(&pdw1000local->cbData);
    }
  }

  // Handle frame reception/preamble detect timeout events
  if (status & SYS_STATUS_ALL_RX_TO) {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXRFTO); // Clear RX timeout event bits

    pdw1000local->wait4resp = 0;

    // Because of an issue with receiver restart after error conditions, an RX reset must be applied after any error or timeout event to ensure
    // the next good frame's timestamp is computed correctly.
    // See section "RX Message timestamp" in DW1000 User Manual.
    dwt_forcetrxoff();
    dwt_rxreset();

    // Call the corresponding callback if present
    if (pdw1000local->cbRxTo != NULL) {
      pdw1000local->cbRxTo(&pdw1000local->cbData);
    }
  }

  // Handle RX errors events
  if (status & SYS_STATUS_ALL_RX_ERR) {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR); // Clear RX error event bits

    pdw1000local->wait4resp = 0;

    // Because of an issue with receiver restart after error conditions, an RX reset must be applied after any error or timeout event to ensure
    // the next good frame's timestamp is computed correctly.
    // See section "RX Message timestamp" in DW1000 User Manual.
    dwt_forcetrxoff();
    dwt_rxreset();

    // Call the corresponding callback if present
    if (pdw1000local->cbRxErr != NULL) {
      pdw1000local->cbRxErr(&pdw1000local->cbData);
    }
  }
}

void dwt_lowpowerlistenisr(void) {
  uint32_t status = pdw1000local->cbData.status = dwt_read32bitreg(SYS_STATUS_ID); // Read status register low 32bits
  uint16_t finfo16;
  uint16_t len;

  // The only interrupt handled when in low-power listening mode is RX good frame so proceed directly to the handling of the received frame.

  // Deactivate low-power listening before clearing the interrupt. If not, the DW1000 will go back to sleep as soon as the interrupt is cleared.
  dwt_setlowpowerlistening(0);

  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_GOOD); // Clear all receive status bits

  pdw1000local->cbData.rx_flags = 0;

  // Read frame info - Only the first two bytes of the register are used here.
  finfo16 = dwt_read16bitoffsetreg(RX_FINFO_ID, 0);

  // Report frame length - Standard frame length up to 127, extended frame length up to 1023 bytes
  len = finfo16 & RX_FINFO_RXFL_MASK_1023;
  if (pdw1000local->longFrames == 0) {
    len &= RX_FINFO_RXFLEN_MASK;
  }
  pdw1000local->cbData.datalength = len;

  // Report ranging bit
  if (finfo16 & RX_FINFO_RNG) {
    pdw1000local->cbData.rx_flags |= DWT_CB_DATA_RX_FLAG_RNG;
  }

  // Report frame control - First bytes of the received frame.
  dwt_readfromdevice(RX_BUFFER_ID, 0, FCTRL_LEN_MAX, pdw1000local->cbData.fctrl);

  // Because of a previous frame not being received properly, AAT bit can be set upon the proper reception of a frame not requesting for
  // acknowledgement (ACK frame is not actually sent though). If the AAT bit is set, check ACK request bit in frame control to confirm (this
  // implementation works only for IEEE802.15.4-2011 compliant frames).
  // This issue is not documented at the time of writing this code. It should be in next release of DW1000 User Manual (v2.09, from July 2016).
  if ((status & SYS_STATUS_AAT) && ((pdw1000local->cbData.fctrl[0] & FCTRL_ACK_REQ_MASK) == 0)) {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_AAT); // Clear AAT status bit in register
    pdw1000local->cbData.status &= ~SYS_STATUS_AAT;   // Clear AAT status bit in callback data register copy
    pdw1000local->wait4resp = 0;
  }

  // Call the corresponding callback if present
  if (pdw1000local->cbRxOk != NULL) {
    pdw1000local->cbRxOk(&pdw1000local->cbData);
  }
}

void dwt_setleds(uint8_t mode) {
  uint32_t reg;

  if (mode & DWT_LEDS_ENABLE) {
    // Set up MFIO for LED output.
    reg = dwt_read32bitoffsetreg(GPIO_CTRL_ID, GPIO_MODE_OFFSET);
    reg &= ~(GPIO_MSGP2_MASK | GPIO_MSGP3_MASK);
    reg |= (GPIO_PIN2_RXLED | GPIO_PIN3_TXLED);
    dwt_write32bitoffsetreg(GPIO_CTRL_ID, GPIO_MODE_OFFSET, reg);

    // Enable LP Oscillator to run from counter and turn on de-bounce clock.
    reg = dwt_read32bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET);
    reg |= (PMSC_CTRL0_GPDCE | PMSC_CTRL0_KHZCLEN);
    dwt_write32bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET, reg);

    // Enable LEDs to blink and set default blink time.
    reg = PMSC_LEDC_BLNKEN | PMSC_LEDC_BLINK_TIME_DEF;
    // Make LEDs blink once if requested.
    if (mode & DWT_LEDS_INIT_BLINK) {
      reg |= PMSC_LEDC_BLINK_NOW_ALL;
    }
    dwt_write32bitoffsetreg(PMSC_ID, PMSC_LEDC_OFFSET, reg);
    // Clear force blink bits if needed.
    if (mode & DWT_LEDS_INIT_BLINK) {
      reg &= ~PMSC_LEDC_BLINK_NOW_ALL;
      dwt_write32bitoffsetreg(PMSC_ID, PMSC_LEDC_OFFSET, reg);
    }
  } else {
    // Clear the GPIO bits that are used for LED control.
    reg = dwt_read32bitoffsetreg(GPIO_CTRL_ID, GPIO_MODE_OFFSET);
    reg &= ~(GPIO_MSGP2_MASK | GPIO_MSGP3_MASK);
    dwt_write32bitoffsetreg(GPIO_CTRL_ID, GPIO_MODE_OFFSET, reg);
  }
}

void _dwt_enableclocks(int clocks) {
  uint8_t reg[2];

  dwt_readfromdevice(PMSC_ID, PMSC_CTRL0_OFFSET, 2, reg);
  switch (clocks) {
  case ENABLE_ALL_SEQ: {
    reg[0] = 0x00;
    reg[1] = reg[1] & 0xfe;
  } break;
  case FORCE_SYS_XTI: {
    // System and RX
    reg[0] = 0x01 | (reg[0] & 0xfc);
  } break;
  case FORCE_SYS_PLL: {
    // System
    reg[0] = 0x02 | (reg[0] & 0xfc);
  } break;
  case READ_ACC_ON: {
    reg[0] = 0x48 | (reg[0] & 0xb3);
    reg[1] = 0x80 | reg[1];
  } break;
  case READ_ACC_OFF: {
    reg[0] = reg[0] & 0xb3;
    reg[1] = 0x7f & reg[1];
  } break;
  case FORCE_OTP_ON: {
    reg[1] = 0x02 | reg[1];
  } break;
  case FORCE_OTP_OFF: {
    reg[1] = reg[1] & 0xfd;
  } break;
  case FORCE_TX_PLL: {
    reg[0] = 0x20 | (reg[0] & 0xcf);
  } break;
  case FORCE_LDE: {
    reg[0] = 0x01;
    reg[1] = 0x03;
  } break;
  default:
    break;
  }

  // Need to write lower byte separately before setting the higher byte(s)
  dwt_writetodevice(PMSC_ID, PMSC_CTRL0_OFFSET, 1, &reg[0]);
  dwt_writetodevice(PMSC_ID, 0x1, 1, &reg[1]);

} // end _dwt_enableclocks()

void _dwt_disablesequencing(void)                                                 // Disable sequencing and go to state "INIT"
{
  _dwt_enableclocks(FORCE_SYS_XTI);                                               // Set system clock to XTI

  dwt_write16bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET, PMSC_CTRL1_PKTSEQ_DISABLE); // Disable PMSC ctrl of RF and RX clk blocks
}

void dwt_setdelayedtrxtime(uint32_t starttime) {
  dwt_write32bitoffsetreg(DX_TIME_ID, 1, starttime); // Write at offset 1 as the lower 9 bits of this register are ignored

} // end dwt_setdelayedtrxtime()

int dwt_starttx(uint8_t mode) {
  int retval         = DWT_SUCCESS;
  uint8_t temp       = 0x00;
  uint16_t checkTxOK = 0;

  if (mode & DWT_RESPONSE_EXPECTED) {
    temp                    = (uint8_t)SYS_CTRL_WAIT4RESP; // Set wait4response bit
    pdw1000local->wait4resp = 1;
  }

  if (mode & DWT_START_TX_DELAYED) {
    // Both SYS_CTRL_TXSTRT and SYS_CTRL_TXDLYS to correctly enable TX
    temp |= (uint8_t)(SYS_CTRL_TXDLYS | SYS_CTRL_TXSTRT);
    dwt_write8bitoffsetreg(SYS_CTRL_ID, SYS_CTRL_OFFSET, temp);
    checkTxOK = dwt_read16bitoffsetreg(SYS_STATUS_ID, 3); // Read at offset 3 to get the upper 2 bytes out of 5
    if ((checkTxOK & SYS_STATUS_TXERR) == 0)              // Transmit Delayed Send set over Half a Period away or Power Up error (there is enough time to send but not to power up individual blocks).
    {
      retval = DWT_SUCCESS;                               // All okay
    } else {
      // If HPDWARN or TXPUTE are set this indicates that the TXDLYS was set too late for the specified DX_TIME.
      // remedial action is to cancel delayed send and report error
      dwt_write8bitoffsetreg(SYS_CTRL_ID, SYS_CTRL_OFFSET, (uint8_t)SYS_CTRL_TRXOFF);
      retval = DWT_ERROR; // Failed !
    }
  } else {
    temp |= (uint8_t)SYS_CTRL_TXSTRT;
    dwt_write8bitoffsetreg(SYS_CTRL_ID, SYS_CTRL_OFFSET, temp);
  }

  return retval;

} // end dwt_starttx()

void dwt_forcetrxoff(void) {
  decaIrqStatus_t stat;
  uint32_t mask;

  mask = dwt_read32bitreg(SYS_MASK_ID); // Read set interrupt mask

  // Need to beware of interrupts occurring in the middle of following read modify write cycle
  // We can disable the radio, but before the status is cleared an interrupt can be set (e.g. the
  // event has just happened before the radio was disabled)
  // thus we need to disable interrupt during this operation
  stat = decamutexon();

  dwt_write32bitreg(SYS_MASK_ID, 0);                                              // Clear interrupt mask - so we don't get any unwanted events

  dwt_write8bitoffsetreg(SYS_CTRL_ID, SYS_CTRL_OFFSET, (uint8_t)SYS_CTRL_TRXOFF); // Disable the radio

  // Forcing Transceiver off - so we do not want to see any new events that may have happened
  dwt_write32bitreg(SYS_STATUS_ID, (SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_GOOD));

  dwt_syncrxbufptrs();

  dwt_write32bitreg(SYS_MASK_ID, mask); // Set interrupt mask to what it was

  // Enable/restore interrupts again...
  decamutexoff(stat);
  pdw1000local->wait4resp = 0;

} // end deviceforcetrxoff()

void dwt_syncrxbufptrs(void) {
  uint8_t buff;
  // Need to make sure that the host/IC buffer pointers are aligned before starting RX
  buff = dwt_read8bitoffsetreg(SYS_STATUS_ID, 3);                    // Read 1 byte at offset 3 to get the 4th byte out of 5

  if ((buff & (SYS_STATUS_ICRBP >> 24)) !=                           // IC side Receive Buffer Pointer
      ((buff & (SYS_STATUS_HSRBP >> 24)) << 1))                      // Host Side Receive Buffer Pointer
  {
    dwt_write8bitoffsetreg(SYS_CTRL_ID, SYS_CTRL_HRBT_OFFSET, 0x01); // We need to swap RX buffer status reg (write one to toggle internally)
  }
}

void dwt_setsniffmode(int enable, uint8_t timeOn, uint8_t timeOff) {
  uint32_t pmsc_reg;
  if (enable) {
    /* Configure ON/OFF times and enable PLL2 on/off sequencing by SNIFF mode. */
    uint16_t sniff_reg = (((uint16_t)timeOff << 8) | timeOn) & RX_SNIFF_MASK;
    dwt_write16bitoffsetreg(RX_SNIFF_ID, RX_SNIFF_OFFSET, sniff_reg);
    pmsc_reg = dwt_read32bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET);
    pmsc_reg |= PMSC_CTRL0_PLL2_SEQ_EN;
    dwt_write32bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET, pmsc_reg);
  } else {
    /* Clear ON/OFF times and disable PLL2 on/off sequencing by SNIFF mode. */
    dwt_write16bitoffsetreg(RX_SNIFF_ID, RX_SNIFF_OFFSET, 0x0000);
    pmsc_reg = dwt_read32bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET);
    pmsc_reg &= ~PMSC_CTRL0_PLL2_SEQ_EN;
    dwt_write32bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET, pmsc_reg);
  }
}

void dwt_setlowpowerlistening(int enable) {
  uint32_t pmsc_reg = dwt_read32bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET);
  if (enable) {
    /* Configure RX to sleep and snooze features. */
    pmsc_reg |= (PMSC_CTRL1_ARXSLP | PMSC_CTRL1_SNOZE);
  } else {
    /* Reset RX to sleep and snooze features. */
    pmsc_reg &= ~(PMSC_CTRL1_ARXSLP | PMSC_CTRL1_SNOZE);
  }
  dwt_write32bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET, pmsc_reg);
}

void dwt_setsnoozetime(uint8_t snooze_time) {
  dwt_write8bitoffsetreg(PMSC_ID, PMSC_SNOZT_OFFSET, snooze_time);
}

int dwt_rxenable(int mode) {
  uint16_t temp;
  uint8_t temp1;

  if ((mode & DWT_NO_SYNC_PTRS) == 0) {
    dwt_syncrxbufptrs();
  }

  temp = (uint16_t)SYS_CTRL_RXENAB;

  if (mode & DWT_START_RX_DELAYED) {
    temp |= (uint16_t)SYS_CTRL_RXDLYE;
  }

  dwt_write16bitoffsetreg(SYS_CTRL_ID, SYS_CTRL_OFFSET, temp);

  if (mode & DWT_START_RX_DELAYED)                   // check for errors
  {
    temp1 = dwt_read8bitoffsetreg(SYS_STATUS_ID, 3); // Read 1 byte at offset 3 to get the 4th byte out of 5
    if ((temp1 & (SYS_STATUS_HPDWARN >> 24)) != 0)   // if delay has passed do immediate RX on unless DWT_IDLE_ON_DLY_ERR is true
    {
      dwt_forcetrxoff();                             // turn the delayed receive off

      if ((mode & DWT_IDLE_ON_DLY_ERR) == 0)         // if DWT_IDLE_ON_DLY_ERR not set then re-enable receiver
      {
        dwt_write16bitoffsetreg(SYS_CTRL_ID, SYS_CTRL_OFFSET, SYS_CTRL_RXENAB);
      }
      return DWT_ERROR; // return warning indication
    }
  }

  return DWT_SUCCESS;
} // end dwt_rxenable()

void dwt_setrxtimeout(uint16_t time) {
  uint8_t temp;

  temp = dwt_read8bitoffsetreg(SYS_CFG_ID, 3); // Read at offset 3 to get the upper byte only

  if (time > 0) {
    dwt_write16bitoffsetreg(RX_FWTO_ID, RX_FWTO_OFFSET, time);

    temp |= (uint8_t)(SYS_CFG_RXWTOE >> 24); // Shift RXWTOE mask as we read the upper byte only
    // OR in 32bit value (1 bit set), I know this is in high byte.
    pdw1000local->sysCFGreg |= SYS_CFG_RXWTOE;

    dwt_write8bitoffsetreg(SYS_CFG_ID, 3, temp); // Write at offset 3 to write the upper byte only
  } else {
    temp &= ~((uint8_t)(SYS_CFG_RXWTOE >> 24));  // Shift RXWTOE mask as we read the upper byte only
    // AND in inverted 32bit value (1 bit clear), I know this is in high byte.
    pdw1000local->sysCFGreg &= ~(SYS_CFG_RXWTOE);

    dwt_write8bitoffsetreg(SYS_CFG_ID, 3, temp); // Write at offset 3 to write the upper byte only
  }

} // end dwt_setrxtimeout()

void dwt_setpreambledetecttimeout(uint16_t timeout) {
  dwt_write16bitoffsetreg(DRX_CONF_ID, DRX_PRETOC_OFFSET, timeout);
}

void dwt_setinterrupt(uint32_t bitmask, uint8_t operation) {
  decaIrqStatus_t stat;
  uint32_t mask;

  // Need to beware of interrupts occurring in the middle of following read modify write cycle
  stat = decamutexon();

  if (operation == 2) {
    dwt_write32bitreg(SYS_MASK_ID, bitmask); // New value
  } else {
    mask = dwt_read32bitreg(SYS_MASK_ID);    // Read register
    if (operation == 1) {
      mask |= bitmask;
    } else {
      mask &= ~bitmask;                   // Clear the bit
    }
    dwt_write32bitreg(SYS_MASK_ID, mask); // New value
  }

  decamutexoff(stat);
}

void dwt_configeventcounters(int enable) {
  // Need to clear and disable, can't just clear
  dwt_write8bitoffsetreg(DIG_DIAG_ID, EVC_CTRL_OFFSET, (uint8_t)(EVC_CLR));

  if (enable) {
    dwt_write8bitoffsetreg(DIG_DIAG_ID, EVC_CTRL_OFFSET, (uint8_t)(EVC_EN)); // Enable
  }
}

void dwt_readeventcounters(dwt_deviceentcnts_t* counters) {
  uint32_t temp;

  temp          = dwt_read32bitoffsetreg(DIG_DIAG_ID, EVC_PHE_OFFSET); // Read sync loss (31-16), PHE (15-0)
  counters->PHE = temp & 0xFFF;
  counters->RSL = (temp >> 16) & 0xFFF;

  temp           = dwt_read32bitoffsetreg(DIG_DIAG_ID, EVC_FCG_OFFSET); // Read CRC bad (31-16), CRC good (15-0)
  counters->CRCG = temp & 0xFFF;
  counters->CRCB = (temp >> 16) & 0xFFF;

  temp           = dwt_read32bitoffsetreg(DIG_DIAG_ID, EVC_FFR_OFFSET); // Overruns (31-16), address errors (15-0)
  counters->ARFE = temp & 0xFFF;
  counters->OVER = (temp >> 16) & 0xFFF;

  temp            = dwt_read32bitoffsetreg(DIG_DIAG_ID, EVC_STO_OFFSET); // Read PTO (31-16), SFDTO (15-0)
  counters->PTO   = (temp >> 16) & 0xFFF;
  counters->SFDTO = temp & 0xFFF;

  temp          = dwt_read32bitoffsetreg(DIG_DIAG_ID, EVC_FWTO_OFFSET); // Read RX TO (31-16), TXFRAME (15-0)
  counters->TXF = (temp >> 16) & 0xFFF;
  counters->RTO = temp & 0xFFF;

  temp          = dwt_read32bitoffsetreg(DIG_DIAG_ID, EVC_HPW_OFFSET); // Read half period warning events
  counters->HPW = temp & 0xFFF;
  counters->TXW = (temp >> 16) & 0xFFF;                                // Power-up warning events
}

void dwt_rxreset(void) {
  // Set RX reset
  dwt_write8bitoffsetreg(PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, PMSC_CTRL0_RESET_RX);

  // Clear RX reset
  dwt_write8bitoffsetreg(PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, PMSC_CTRL0_RESET_CLEAR);
}

void dwt_softreset(void) {
  _dwt_disablesequencing();

  // Clear any AON auto download bits (as reset will trigger AON download)
  dwt_write16bitoffsetreg(AON_ID, AON_WCFG_OFFSET, 0x00);
  // Clear the wake-up configuration
  dwt_write8bitoffsetreg(AON_ID, AON_CFG0_OFFSET, 0x00);
  // Upload the new configuration
  _dwt_aonarrayupload();

  // Reset HIF, TX, RX and PMSC (set the reset bits)
  dwt_write8bitoffsetreg(PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, PMSC_CTRL0_RESET_ALL);

  // DW1000 needs a 10us sleep to let clk PLL lock after reset - the PLL will automatically lock after the reset
  // Could also have polled the PLL lock flag, but then the SPI needs to be < 3MHz !! So a simple delay is easier
  deca_sleep(1);

  // Clear the reset bits
  dwt_write8bitoffsetreg(PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, PMSC_CTRL0_RESET_CLEAR);

  pdw1000local->wait4resp = 0;
}

void dwt_setxtaltrim(uint8_t value) {
  // The 3 MSb in this 8-bit register must be kept to 0b011 to avoid any malfunction.
  uint8_t reg_val = (3 << 5) | (value & FS_XTALT_MASK);
  dwt_write8bitoffsetreg(FS_CTRL_ID, FS_XTALT_OFFSET, reg_val);
}

uint8_t dwt_getxtaltrim(void) {
  return (dwt_read8bitoffsetreg(FS_CTRL_ID, FS_XTALT_OFFSET) & FS_XTALT_MASK);
}

void dwt_configcwmode(uint8_t chan) {
#ifdef DWT_API_ERROR_CHECK
  assert((chan >= 1) && (chan <= 7) && (chan != 6));
#endif

  // Disable TX/RX RF block sequencing (needed for cw frame mode)
  _dwt_disablesequencing();

  // Config RF pll (for a given channel)
  // Configure PLL2/RF PLL block CFG/TUNE
  dwt_write32bitoffsetreg(FS_CTRL_ID, FS_PLLCFG_OFFSET, fs_pll_cfg[chan_idx[chan]]);
  dwt_write8bitoffsetreg(FS_CTRL_ID, FS_PLLTUNE_OFFSET, fs_pll_tune[chan_idx[chan]]);
  // PLL wont be enabled until a TX/RX enable is issued later on
  // Configure RF TX blocks (for specified channel and prf)
  // Config RF TX control
  dwt_write32bitoffsetreg(RF_CONF_ID, RF_TXCTRL_OFFSET, tx_config[chan_idx[chan]]);

  // Enable RF PLL
  dwt_write32bitreg(RF_CONF_ID, RF_CONF_TXPLLPOWEN_MASK); // Enable LDO and RF PLL blocks
  dwt_write32bitreg(RF_CONF_ID, RF_CONF_TXALLEN_MASK);    // Enable the rest of TX blocks

  // Configure TX clocks
  dwt_write8bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET, 0x22);
  dwt_write8bitoffsetreg(PMSC_ID, 0x1, 0x07);

  // Disable fine grain TX sequencing
  dwt_setfinegraintxseq(0);

  // Configure CW mode
  dwt_write8bitoffsetreg(TX_CAL_ID, TC_PGTEST_OFFSET, TC_PGTEST_CW);
}

void dwt_configcontinuousframemode(uint32_t framerepetitionrate) {
  //
  // Disable TX/RX RF block sequencing (needed for continuous frame mode)
  //
  _dwt_disablesequencing();

  //
  // Enable RF PLL and TX blocks
  //
  dwt_write32bitreg(RF_CONF_ID, RF_CONF_TXPLLPOWEN_MASK); // Enable LDO and RF PLL blocks
  dwt_write32bitreg(RF_CONF_ID, RF_CONF_TXALLEN_MASK);    // Enable the rest of TX blocks

  //
  // Configure TX clocks
  //
  _dwt_enableclocks(FORCE_SYS_PLL);
  _dwt_enableclocks(FORCE_TX_PLL);

  // Set the frame repetition rate
  if (framerepetitionrate < 4) {
    framerepetitionrate = 4;
  }
  dwt_write32bitreg(DX_TIME_ID, framerepetitionrate);

  //
  // Configure continuous frame TX
  //
  dwt_write8bitoffsetreg(DIG_DIAG_ID, DIAG_TMC_OFFSET, (uint8_t)(DIAG_TMC_TX_PSTM)); // Turn the tx power spectrum test mode - continuous sending of frames
}

uint16_t dwt_readtempvbat(uint8_t fastSPI) {
  uint8_t wr_buf[2];
  uint8_t vbat_raw;
  uint8_t temp_raw;

  // These writes should be single writes and in sequence
  wr_buf[0] = 0x80; // Enable TLD Bias
  dwt_writetodevice(RF_CONF_ID, 0x11, 1, wr_buf);

  wr_buf[0] = 0x0A; // Enable TLD Bias and ADC Bias
  dwt_writetodevice(RF_CONF_ID, 0x12, 1, wr_buf);

  wr_buf[0] = 0x0f;                               // Enable Outputs (only after Biases are up and running)
  dwt_writetodevice(RF_CONF_ID, 0x12, 1, wr_buf); //

  if (fastSPI == 1) {
    // Reading All SAR inputs
    wr_buf[0] = 0x00;
    dwt_writetodevice(TX_CAL_ID, TC_SARL_SAR_C, 1, wr_buf);
    wr_buf[0] = 0x01; // Set SAR enable
    dwt_writetodevice(TX_CAL_ID, TC_SARL_SAR_C, 1, wr_buf);

    deca_sleep(1); // If using PLL clocks(and fast SPI rate) then this sleep is needed
    // Read voltage and temperature.
    dwt_readfromdevice(TX_CAL_ID, TC_SARL_SAR_LVBAT_OFFSET, 2, wr_buf);
  } else                              // change to a slow clock
  {
    _dwt_enableclocks(FORCE_SYS_XTI); // NOTE: set system clock to XTI - this is necessary to make sure the values read are reliable
    // Reading All SAR inputs
    wr_buf[0] = 0x00;
    dwt_writetodevice(TX_CAL_ID, TC_SARL_SAR_C, 1, wr_buf);
    wr_buf[0] = 0x01; // Set SAR enable
    dwt_writetodevice(TX_CAL_ID, TC_SARL_SAR_C, 1, wr_buf);

    // Read voltage and temperature.
    dwt_readfromdevice(TX_CAL_ID, TC_SARL_SAR_LVBAT_OFFSET, 2, wr_buf);
    // Default clocks (ENABLE_ALL_SEQ)
    _dwt_enableclocks(ENABLE_ALL_SEQ); // Enable clocks for sequencing
  }

  vbat_raw = wr_buf[0];
  temp_raw = wr_buf[1];

  wr_buf[0] = 0x00; // Clear SAR enable
  dwt_writetodevice(TX_CAL_ID, TC_SARL_SAR_C, 1, wr_buf);

  return (((uint16_t)temp_raw << 8) | (vbat_raw));
}

float dwt_convertrawtemperature(uint8_t raw_temp) {
  float realtemp;
#ifdef DWT_API_ERROR_CHECK
  assert(pdw1000local->otp_mask & DWT_READ_OTP_TMP);
#endif
  // the User Manual formula is: Temperature (�C) = ( (SAR_LTEMP � OTP_READ(Vtemp @ 23�C) ) x 1.14) + 23
  realtemp = ((raw_temp - pdw1000local->tempP) * SAR_TEMP_TO_CELCIUS_CONV) + 23;

  return realtemp;
}

uint8_t dwt_convertdegtemptoraw(int16_t externaltemp) {
  int32_t raw_temp;
#ifdef DWT_API_ERROR_CHECK
  assert(pdw1000local->otp_mask & DWT_READ_OTP_TMP);
  assert((externaltemp > -800) && (externaltemp < 1500))
#endif
      // the User Manual formula is: Temperature (�C) = ( (SAR_LTEMP � OTP_READ(Vtemp @ 23�C) ) x 1.14) + 23
      raw_temp
      = ((externaltemp - 230 + 5) * DCELCIUS_TO_SAR_TEMP_CONV); //+5 for better rounding

  if (raw_temp < 0)                                             // negative
  {
    raw_temp = (-raw_temp >> 8);
    raw_temp = -raw_temp;
  } else
    raw_temp = raw_temp >> 8;

  return (uint8_t)(raw_temp + pdw1000local->tempP);
}

float dwt_convertrawvoltage(uint8_t raw_voltage) {
  float realvolt;

#ifdef DWT_API_ERROR_CHECK
  assert(pdw1000local->otp_mask & DWT_READ_OTP_BAT);
#endif
  // the User Manual formula is: Voltage (V) = ( (SAR_LVBAT � OTP_READ(Vmeas @ 3.3 V) ) / 173 ) + 3.3
  realvolt = ((float)(raw_voltage - pdw1000local->vBatP) * SAR_VBAT_TO_VOLT_CONV) + 3.3;

  return realvolt;
}

uint8_t dwt_convertvoltstoraw(int32_t externalmvolt) {
  uint32_t raw_voltage;
#ifdef DWT_API_ERROR_CHECK
  assert(pdw1000local->otp_mask & DWT_READ_OTP_BAT);
#endif
  // the User Manual formula is: Voltage (V) = ( (SAR_LVBAT � OTP_READ(Vmeas @ 3.3 V) ) / 173 ) + 3.3
  raw_voltage = ((externalmvolt - 3300) * MVOLT_TO_SAR_VBAT_CONV) + pdw1000local->vBatP;

  return (uint8_t)raw_voltage;
}

uint8_t dwt_readwakeuptemp(void) {
  return dwt_read8bitoffsetreg(TX_CAL_ID, TC_SARL_SAR_LTEMP_OFFSET);
}

uint8_t dwt_readwakeupvbat(void) {
  return dwt_read8bitoffsetreg(TX_CAL_ID, TC_SARL_SAR_LVBAT_OFFSET);
}

uint32_t dwt_calcbandwidthtempadj(uint16_t target_count) {
  int i;
  uint8_t bit_field, curr_bw;
  int32_t delta_count = 0;
  uint32_t best_bw    = 0;
  uint16_t raw_count  = 0;
  int32_t delta_lowest;

  // Used to store the current values of the registers so that they can be restored after
  uint8_t old_pmsc_ctrl0;
  uint16_t old_pmsc_ctrl1;
  uint32_t old_rf_conf_txpow_mask;

  // Record the current values of these registers, to restore later
  old_pmsc_ctrl0         = dwt_read8bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET);
  old_pmsc_ctrl1         = dwt_read16bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET);
  old_rf_conf_txpow_mask = dwt_read32bitreg(RF_CONF_ID);

  //  Set clock to XTAL
  dwt_write8bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET, PMSC_CTRL0_SYSCLKS_19M);

  //  Disable sequencing
  dwt_write16bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET, PMSC_CTRL1_PKTSEQ_DISABLE);

  //  Turn on CLK PLL, Mix Bias and PG
  dwt_write32bitreg(RF_CONF_ID, RF_CONF_TXPOW_MASK | RF_CONF_PGMIXBIASEN_MASK);

  //  Set sys and TX clock to PLL
  dwt_write8bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET, PMSC_CTRL0_SYSCLKS_125M | PMSC_CTRL0_TXCLKS_125M);

  // Set the MSB high for first guess
  curr_bw      = 0x80;
  // Set starting bit
  bit_field    = 0x80;
  // Initial lowest delta is the maximum difference that we should allow the count value to be from the target.
  // If the algorithm is successful, it will be overwritten by a smaller value where the count value is closer
  // to the target
  delta_lowest = 300;

  for (i = 0; i < 7; i++) {
    // start with 0xc0 and test.
    bit_field = bit_field >> 1;
    curr_bw   = curr_bw | bit_field;

    // Write bw setting to PG_DELAY register
    dwt_write8bitoffsetreg(TX_CAL_ID, TC_PGDELAY_OFFSET, curr_bw);

    // Set cal direction and time
    dwt_write8bitoffsetreg(TX_CAL_ID, TC_PGCCTRL_OFFSET, TC_PGCCTRL_DIR_CONV | TC_PGCCTRL_TMEAS_MASK);

    // Start cal
    dwt_write8bitoffsetreg(TX_CAL_ID, TC_PGCCTRL_OFFSET, TC_PGCCTRL_DIR_CONV | TC_PGCCTRL_TMEAS_MASK | TC_PGCCTRL_CALSTART);
    // Allow cal to complete
    deca_sleep(1);

    // Read count value from the PG cal block
    raw_count = dwt_read16bitoffsetreg(TX_CAL_ID, TC_PGCAL_STATUS_OFFSET) & TC_PGCAL_STATUS_DELAY_MASK;

    // lets keep track of the closest value to the target in case we overshoot
    delta_count = abs((int)raw_count - (int)target_count);
    if (delta_count < delta_lowest) {
      delta_lowest = delta_count;
      best_bw      = curr_bw;
    }

    // Test the count results
    if (raw_count > target_count)
      // Count was lower, BW was lower so increase PG DELAY
      curr_bw = curr_bw | bit_field;
    else
      // Count was higher
      curr_bw = curr_bw & (~(bit_field));
  }

  // Restore old register values
  dwt_write8bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET, old_pmsc_ctrl0);
  dwt_write16bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET, old_pmsc_ctrl1);
  dwt_write32bitreg(RF_CONF_ID, old_rf_conf_txpow_mask);

  // Returns the best PG_DELAY setting
  return best_bw;
}

uint32_t _dwt_computetxpowersetting(uint32_t ref_powerreg, int32_t power_adj) {
  int8_t da_attn_change, mixer_gain_change;
  uint8_t current_da_attn, current_mixer_gain;
  uint8_t new_da_attn, new_mixer_gain;
  uint32_t new_regval = 0;
  int i;

  for (i = 0; i < 4; i++) {
    da_attn_change     = 0;
    mixer_gain_change  = power_adj;
    current_da_attn    = ((ref_powerreg >> (i * 8)) & 0xE0) >> 5;
    current_mixer_gain = (ref_powerreg >> (i * 8)) & 0x1F;

    // Mixer gain gives best performance between gain value of 4 and 20
    while ((current_mixer_gain + mixer_gain_change < 4) || (current_mixer_gain + mixer_gain_change > 20)) {
      // If mixer gain goes outside bounds, adjust the DA attenuation to compensate
      if (current_mixer_gain + mixer_gain_change > 20) {
        da_attn_change -= 1;

        if (da_attn_change == 0) // DA attenuation has reached the max value
        {
          da_attn_change = 1;    // restore the value and exit the loop - DA is at max allowed
          break;
        }

        mixer_gain_change -= (int8_t)(MIX_DA_FACTOR);
      } else if (current_mixer_gain + mixer_gain_change < 4) {
        da_attn_change += 1;

        if (da_attn_change == 0x8) // DA attenuation has reached the min value
        {
          da_attn_change = 7;      // restore the value and exit the loop - DA is at min allowed
          break;
        }

        mixer_gain_change += (int8_t)(MIX_DA_FACTOR);
      }
    }

    new_da_attn    = (current_da_attn + da_attn_change) & 0x7;
    new_mixer_gain = (current_mixer_gain + mixer_gain_change) & 0x1F;

    new_regval |= ((uint32_t)((new_da_attn << 5) | new_mixer_gain)) << (i * 8);
  }

  return (uint32_t)new_regval;
}

uint32_t dwt_calcpowertempadj(uint8_t channel, uint32_t ref_powerreg, int delta_temp) {
  int8_t delta_power;
  int negative = 0;

  if (delta_temp < 0) {
    negative   = 1;
    delta_temp = -delta_temp; // make (-)ve into (+)ve number
  }

  // Calculate the expected power differential at the current temperature
  if (channel == 5) {
    delta_power = ((delta_temp * TEMP_COMP_FACTOR_CH5) >> 12); //>>12 is same as /4096
  } else if (channel == 2) {
    delta_power = ((delta_temp * TEMP_COMP_FACTOR_CH2) >> 12); //>>12 is same as /4096
  } else
    delta_power = 0;

  if (negative == 1) {
    delta_power = -delta_power; // restore the sign
  }

  if (delta_power == 0)
    return ref_powerreg; // no change to power register

  // Adjust the TX_POWER register value
  return _dwt_computetxpowersetting(ref_powerreg, delta_power);
}

uint16_t dwt_calcpgcount(uint8_t pgdly) {
  // Perform PG count read ten times and take an average to smooth out any noise
  const int NUM_SAMPLES  = 10;
  uint32_t sum_count     = 0;
  uint16_t average_count = 0, count = 0;
  int i = 0;

  // Used to store the current values of the registers so that they can be restored after
  uint8_t old_pmsc_ctrl0;
  uint16_t old_pmsc_ctrl1;
  uint32_t old_rf_conf_txpow_mask;

  // Record the current values of these registers, to restore later
  old_pmsc_ctrl0         = dwt_read8bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET);
  old_pmsc_ctrl1         = dwt_read16bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET);
  old_rf_conf_txpow_mask = dwt_read32bitreg(RF_CONF_ID);

  //  Set clock to XTAL
  dwt_write8bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET, PMSC_CTRL0_SYSCLKS_19M);
  //  Disable sequencing
  dwt_write16bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET, PMSC_CTRL1_PKTSEQ_DISABLE);
  //  Turn on CLK PLL, Mix Bias and PG
  dwt_write32bitreg(RF_CONF_ID, RF_CONF_TXPOW_MASK | RF_CONF_PGMIXBIASEN_MASK);
  //  Set sys and TX clock to PLL
  dwt_write8bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET, PMSC_CTRL0_SYSCLKS_125M | PMSC_CTRL0_TXCLKS_125M);

  for (i = 0; i < NUM_SAMPLES; i++) {
    // Write bw setting to PG_DELAY register
    dwt_write8bitoffsetreg(TX_CAL_ID, TC_PGDELAY_OFFSET, pgdly);

    // Set cal direction and time
    dwt_write8bitoffsetreg(TX_CAL_ID, TC_PGCCTRL_OFFSET, TC_PGCCTRL_DIR_CONV | TC_PGCCTRL_TMEAS_MASK);

    // Start cal
    dwt_write8bitoffsetreg(TX_CAL_ID, TC_PGCCTRL_OFFSET, TC_PGCCTRL_DIR_CONV | TC_PGCCTRL_TMEAS_MASK | TC_PGCCTRL_CALSTART);

    // Allow cal to complete - the TC_PGCCTRL_CALSTART bit will clear automatically
    deca_sleep(1);

    // Read count value from the PG cal block
    count = dwt_read16bitoffsetreg(TX_CAL_ID, TC_PGCAL_STATUS_OFFSET) & TC_PGCAL_STATUS_DELAY_MASK;

    sum_count += count;
  }

  // Restore old register values
  dwt_write8bitoffsetreg(PMSC_ID, PMSC_CTRL0_OFFSET, old_pmsc_ctrl0);
  dwt_write16bitoffsetreg(PMSC_ID, PMSC_CTRL1_OFFSET, old_pmsc_ctrl1);
  dwt_write32bitreg(RF_CONF_ID, old_rf_conf_txpow_mask);

  average_count = (int)(sum_count / NUM_SAMPLES);
  return average_count;
}

/* ===============================================================================================
   List of expected (known) device ID handled by this software
   ===============================================================================================

    0xDECA0130                               // DW1000 - MP

   ===============================================================================================
*/
